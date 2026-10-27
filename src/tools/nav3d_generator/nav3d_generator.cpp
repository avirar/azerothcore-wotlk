/*
 * This file is part of the AzerothCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If not,
 * see <http://www.gnu.org/licenses/>.
 */

#include "Nav3DTile.h"
#include "StringFormat.h"
#include "mmaps_generator/TerrainBuilder.h"

#include "Recast.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace
{
    constexpr float kDefaultCellSize = 1.0f;
    constexpr float kMinCellSize = 0.5f;
    constexpr float kMaxCellSize = 4.0f;
    constexpr uint32 kGeneratorVersion = 1;
    constexpr size_t kMaxVoxelCells = 512ull * 1024ull * 1024ull;

    struct Options
    {
        std::string DataDir = "env/dist/bin";
        std::string OutputDir;
        uint32 MapId = 0;
        float CellSize = kDefaultCellSize;
        bool HasCenter = false;
        float CenterX = 0.0f;
        float CenterY = 0.0f;
        float Radius = 0.0f;
        bool HasWaterLevel = false;
        float WaterLevel = 0.0f;
        bool HasMinZ = false;
        float MinZ = 0.0f;
        std::set<std::pair<uint32, uint32>> Tiles;
    };

    void PrintUsage(char const* program)
    {
        std::cout << "Usage: " << program << " --data-dir <dir> [options]\n"
                  << "  --output <dir>       Output directory (default: <data-dir>/nav3d)\n"
                  << "  --map <id>           Map id (default: 0)\n"
                  << "  --cell-size <yd>     Grid cell size (default: 1.0)\n"
                  << "  --tile <x:y>         Tile coordinate; repeatable\n"
                  << "  --center-x <x>       Generate tiles around this world X\n"
                  << "  --center-y <y>       Generate tiles around this world Y\n"
                  << "  --radius <yd>        Radius around the center (default: 0)\n"
                  << "  --water-level <z>    Fill water up to this Z (default: local liquid surface)\n"
                  << "  --min-z <z>          Lowest Z included in the tile (default: mesh min)\n";
    }

    bool ParseFloat(std::string const& text, float& value)
    {
        try
        {
            size_t used = 0;
            value = std::stof(text, &used);
            return used == text.size();
        }
        catch (...)
        {
            return false;
        }
    }

    bool ParseUInt(std::string const& text, uint32& value)
    {
        try
        {
            size_t used = 0;
            value = uint32(std::stoul(text, &used));
            return used == text.size();
        }
        catch (...)
        {
            return false;
        }
    }

    bool ParseOptions(int argc, char** argv, Options& options)
    {
        for (int i = 1; i < argc; ++i)
        {
            std::string const arg = argv[i];
            auto const next = [&](std::string& value) -> bool
            {
                if (i + 1 >= argc)
                    return false;
                value = argv[++i];
                return true;
            };

            std::string value;
            if (arg == "--data-dir")
            {
                if (!next(options.DataDir))
                    return false;
            }
            else if (arg == "--output")
            {
                if (!next(options.OutputDir))
                    return false;
            }
            else if (arg == "--map")
            {
                if (!next(value) || !ParseUInt(value, options.MapId))
                    return false;
            }
            else if (arg == "--cell-size")
            {
                if (!next(value) || !ParseFloat(value, options.CellSize))
                    return false;
            }
            else if (arg == "--tile")
            {
                if (!next(value))
                    return false;
                size_t const colon = value.find(':');
                if (colon == std::string::npos)
                    return false;
                uint32 tileX = 0;
                uint32 tileY = 0;
                if (!ParseUInt(value.substr(0, colon), tileX) || !ParseUInt(value.substr(colon + 1), tileY))
                    return false;
                options.Tiles.emplace(tileX, tileY);
            }
            else if (arg == "--center-x")
            {
                if (!next(value) || !ParseFloat(value, options.CenterX))
                    return false;
                options.HasCenter = true;
            }
            else if (arg == "--center-y")
            {
                if (!next(value) || !ParseFloat(value, options.CenterY))
                    return false;
                options.HasCenter = true;
            }
            else if (arg == "--radius")
            {
                if (!next(value) || !ParseFloat(value, options.Radius))
                    return false;
            }
            else if (arg == "--water-level")
            {
                if (!next(value) || !ParseFloat(value, options.WaterLevel))
                    return false;
                options.HasWaterLevel = true;
            }
            else if (arg == "--min-z")
            {
                if (!next(value) || !ParseFloat(value, options.MinZ))
                    return false;
                options.HasMinZ = true;
            }
            else if (arg == "--help" || arg == "-h")
            {
                PrintUsage(argv[0]);
                std::exit(0);
            }
            else
            {
                std::cerr << "Unknown option: " << arg << "\n";
                return false;
            }
        }

        options.CellSize = std::clamp(options.CellSize, kMinCellSize, kMaxCellSize);
        if (options.OutputDir.empty())
            options.OutputDir = (std::filesystem::path(options.DataDir) / "nav3d").string();

        return true;
    }

    uint32 TileFromWorld(float coordinate)
    {
        return uint32(std::max<int32>(0, int32(std::floor(32.0f - coordinate / MMAP::GRID_SIZE))));
    }

    struct Bounds
    {
        float MinX = 0.0f;
        float MinY = 0.0f;
        float MinZ = 0.0f;
        float MaxX = 0.0f;
        float MaxY = 0.0f;
        float MaxZ = 0.0f;
    };

    bool MeshBounds(MMAP::MeshData const& mesh, Bounds& bounds)
    {
        bounds.MinX = std::numeric_limits<float>::max();
        bounds.MinY = std::numeric_limits<float>::max();
        bounds.MinZ = std::numeric_limits<float>::max();
        bounds.MaxX = -std::numeric_limits<float>::max();
        bounds.MaxY = -std::numeric_limits<float>::max();
        bounds.MaxZ = -std::numeric_limits<float>::max();
        bool any = false;

        auto const include = [&bounds, &any](G3D::Array<float> const& verts)
        {
            for (int i = 0; i + 2 < verts.size(); i += 3)
            {
                bounds.MinX = std::min(bounds.MinX, verts[i]);
                bounds.MinY = std::min(bounds.MinY, verts[i + 1]);
                bounds.MinZ = std::min(bounds.MinZ, verts[i + 2]);
                bounds.MaxX = std::max(bounds.MaxX, verts[i]);
                bounds.MaxY = std::max(bounds.MaxY, verts[i + 1]);
                bounds.MaxZ = std::max(bounds.MaxZ, verts[i + 2]);
                any = true;
            }
        };

        include(mesh.solidVerts);
        include(mesh.liquidVerts);
        return any;
    }

    float LiquidTop(rcHeightfield const* liquid, int column, float bminZ, float cellSize)
    {
        float top = -std::numeric_limits<float>::max();
        for (rcSpan const* span = liquid->spans[column]; span; span = span->next)
            top = std::max(top, bminZ + float(span->smax) * cellSize);
        return top;
    }

    bool SolidAt(rcSpan const* first, int z)
    {
        for (rcSpan const* span = first; span; span = span->next)
        {
            if (z >= int(span->smin) && z < int(span->smax))
                return true;
        }
        return false;
    }

    bool GenerateTile(Options const& options, uint32 tileX, uint32 tileY)
    {
        MMAP::TerrainBuilder terrainBuilder(options.DataDir, false);
        MMAP::MeshData meshData;
        terrainBuilder.loadMap(options.MapId, tileX, tileY, meshData);
        for (int offsetX = -1; offsetX <= 1; ++offsetX)
        {
            for (int offsetY = -1; offsetY <= 1; ++offsetY)
            {
                int const wmoX = int(tileX) + offsetX;
                int const wmoY = int(tileY) + offsetY;
                if (wmoX < 0 || wmoY < 0 || wmoX >= 64 || wmoY >= 64)
                    continue;
                terrainBuilder.loadVMap(options.MapId, uint32(wmoY), uint32(wmoX), meshData);
            }
        }
        MMAP::TerrainBuilder::cleanVertices(meshData.solidVerts, meshData.solidTris);
        MMAP::TerrainBuilder::cleanVertices(meshData.liquidVerts, meshData.liquidTris);

        if (meshData.solidVerts.size() == 0 || meshData.solidTris.size() == 0 ||
            (!options.HasWaterLevel && !options.HasMinZ &&
             (meshData.liquidVerts.size() == 0 || meshData.liquidTris.size() == 0)))
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (no usable solid/liquid mesh)\n";
            return true;
        }

        Bounds meshBounds;
        if (!MeshBounds(meshData, meshBounds))
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY << " (no bounds)\n";
            return true;
        }

        float const cellSize = options.CellSize;
        float const recastMinX = (32.0f - float(tileX)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        float const recastMinZ = (32.0f - float(tileY)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        int const width = int(std::ceil(MMAP::GRID_SIZE / cellSize));
        int const height = int(std::ceil(MMAP::GRID_SIZE / cellSize));
        float const recastMaxX = recastMinX + float(width) * cellSize;
        float const recastMaxZ = recastMinZ + float(height) * cellSize;

        float const minZ = options.HasMinZ ? options.MinZ : meshBounds.MinY - cellSize;
        float const globalTop = options.HasWaterLevel ? options.WaterLevel : meshBounds.MaxY + cellSize;
        int const sizeZ = std::max(1, int(std::ceil((globalTop - minZ) / cellSize)));
        if (sizeZ > 32767)
        {
            std::cerr << "tile " << tileX << ":" << tileY << " z range too large: " << sizeZ << "\n";
            return false;
        }

        size_t const cellCount = size_t(width) * size_t(height) * size_t(sizeZ);
        if (cellCount > kMaxVoxelCells)
        {
            std::cerr << "tile " << tileX << ":" << tileY << " voxel grid too large: " << cellCount << "\n";
            return false;
        }

        float bmin[3] = {recastMinX, minZ, recastMinZ};
        float bmax[3] = {recastMaxX, minZ + float(sizeZ) * cellSize, recastMaxZ};

        rcContext context;
        rcHeightfield* solid = rcAllocHeightfield();
        rcHeightfield* liquid = rcAllocHeightfield();
        if (!solid || !liquid)
        {
            rcFreeHeightField(solid);
            rcFreeHeightField(liquid);
            return false;
        }

        if (!rcCreateHeightfield(&context, *solid, width, height, bmin, bmax, cellSize, cellSize) ||
            !rcCreateHeightfield(&context, *liquid, width, height, bmin, bmax, cellSize, cellSize))
        {
            rcFreeHeightField(solid);
            rcFreeHeightField(liquid);
            return false;
        }

        int const solidTriangles = meshData.solidTris.size() / 3;
        int const liquidTriangles = meshData.liquidTris.size() / 3;
        std::vector<unsigned char> solidAreas(solidTriangles, RC_WALKABLE_AREA);
        std::vector<unsigned char> liquidAreas(liquidTriangles, RC_WALKABLE_AREA);

        if (!rcRasterizeTriangles(&context, meshData.solidVerts.getCArray(), meshData.solidVerts.size() / 3,
                                  meshData.solidTris.getCArray(), solidAreas.data(), solidTriangles, *solid) ||
            !rcRasterizeTriangles(&context, meshData.liquidVerts.getCArray(), meshData.liquidVerts.size() / 3,
                                  meshData.liquidTris.getCArray(), liquidAreas.data(), liquidTriangles, *liquid))
        {
            rcFreeHeightField(solid);
            rcFreeHeightField(liquid);
            return false;
        }

        std::vector<uint8> water(cellCount, 0);
        std::vector<uint8> distance(cellCount, 0);

        auto const cellIndex = [width, height, sizeZ](int x, int y, int z) -> size_t
        {
            return (size_t(z) * size_t(height) + size_t(y)) * size_t(width) + size_t(x);
        };

        size_t waterColumns = 0;
        size_t waterCells = 0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int const column = y * width + x;
                rcSpan const* first = solid->spans[column];
                if (!first)
                    continue;

                float const localTop = LiquidTop(liquid, column, bmin[1], cellSize);
                float const waterTop = options.HasWaterLevel ? options.WaterLevel : localTop;
                if (waterTop <= bmin[1])
                    continue;

                int const waterTopCell =
                    std::clamp(int(std::floor((waterTop - bmin[1]) / cellSize)), 0, sizeZ - 1);

                // Water is the complement of solid spans up to the water surface. Below the
                // lowest solid span (seabed/hull bottom) is solid/void, not water.
                std::vector<uint8> solidMask(sizeZ, 0);
                for (int z = 0; z < std::min(int(first->smin), sizeZ); ++z)
                    solidMask[z] = 1;
                for (rcSpan const* span = first; span; span = span->next)
                {
                    int const lo = std::max(0, int(span->smin));
                    int const hi = std::min(sizeZ, int(span->smax));
                    for (int z = lo; z < hi; ++z)
                        solidMask[z] = 1;
                }

                bool anyWater = false;
                for (int z = 0; z <= waterTopCell; ++z)
                {
                    if (solidMask[z])
                        continue;
                    water[cellIndex(x, y, z)] = 1;
                    ++waterCells;
                    anyWater = true;
                }
                if (anyWater)
                    ++waterColumns;
            }
        }

        {
            size_t columnsWithSpans = 0;
            size_t columnsWithWater = 0;
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    if (solid->spans[y * width + x])
                        ++columnsWithSpans;
                    bool w = false;
                    for (int z = 0; z < sizeZ && !w; ++z)
                        w = water[cellIndex(x, y, z)] != 0;
                    if (w)
                        ++columnsWithWater;
                }
            }
            std::cout << "debug columns spans=" << columnsWithSpans << " water=" << columnsWithWater
                      << " waterCells=" << waterCells << "\n";
        }

        for (size_t i = 0; i < cellCount; ++i)
            if (water[i])
                distance[i] = 255;

        auto const relax = [&distance, width, height, sizeZ](uint8& value, int x, int y, int z)
        {
            if (x < 0 || y < 0 || z < 0 || x >= width || y >= height || z >= sizeZ)
            {
                value = std::min<uint8>(value, 1);
                return;
            }
            uint8 const neighbor = distance[(size_t(z) * size_t(height) + size_t(y)) * size_t(width) + size_t(x)];
            if (neighbor < 255)
                value = std::min<uint8>(value, uint8(neighbor + 1));
        };

        for (int z = 0; z < sizeZ; ++z)
        {
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                        continue;
                    uint8 value = 255;
                    relax(value, x - 1, y, z);
                    relax(value, x, y - 1, z);
                    relax(value, x, y, z - 1);
                    distance[index] = value;
                }
            }
        }

        for (int z = sizeZ - 1; z >= 0; --z)
        {
            for (int y = height - 1; y >= 0; --y)
            {
                for (int x = width - 1; x >= 0; --x)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                        continue;
                    uint8 value = distance[index];
                    relax(value, x + 1, y, z);
                    relax(value, x, y + 1, z);
                    relax(value, x, y, z + 1);
                    distance[index] = value;
                }
            }
        }

        Nav3DTile tile;
        Nav3DTileHeader& header = tile.Header();
        header.MapId = options.MapId;
        header.TileX = uint16(tileX);
        header.TileY = uint16(tileY);
        header.Medium = Nav3D::MediumWater;
        header.CellSize = cellSize;
        header.OriginX = recastMinX;
        header.OriginY = recastMinZ;
        header.OriginZ = bmin[1];
        header.ExtentX = float(width) * cellSize;
        header.ExtentY = float(height) * cellSize;
        header.ExtentZ = float(sizeZ) * cellSize;
        header.SizeX = uint32(width);
        header.SizeY = uint32(height);
        header.Flags |= Nav3D::FlagHasSolid;
        header.SizeZ = uint32(sizeZ);
        header.GeneratorVersion = kGeneratorVersion;
        header.GeometryHash = 0;

        tile.Columns().resize(size_t(width) * size_t(height));
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                Nav3DTileColumn& column = tile.Columns()[size_t(y) * size_t(width) + size_t(x)];
                int start = -1;
                uint8 minClearance = 255;

                auto const flush = [&](int end)
                {
                    if (start < 0)
                        return;
                    Nav3DTileInterval interval;
                    interval.ZMin = int16(start);
                    interval.ZMax = int16(end);
                    interval.Clearance = minClearance;
                    interval.Flags = 0;
                    column.IntervalOffset = uint32(tile.Intervals().size());
                    column.IntervalCount = uint16(column.IntervalCount + 1);
                    tile.Intervals().push_back(interval);
                    start = -1;
                    minClearance = 255;
                };

                for (int z = 0; z < sizeZ; ++z)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                    {
                        flush(z - 1);
                        continue;
                    }

                    if (start < 0)
                        start = z;
                    uint32 const clearance = std::min<uint32>(
                        255, uint32(float(distance[index]) * cellSize / Nav3D::ClearanceUnit));
                    minClearance = std::min<uint8>(minClearance, uint8(clearance));
                }
                flush(sizeZ - 1);
            }
        }

        std::string const relative = Acore::StringFormat("map{:04}/{}_{}_water.n3t", options.MapId, tileX,
                                                         tileY);
        std::filesystem::path const outputFile = std::filesystem::path(options.OutputDir) / relative;
        std::error_code ec;
        std::filesystem::create_directories(outputFile.parent_path(), ec);
        if (ec)
        {
            std::cerr << "cannot create " << outputFile.parent_path() << ": " << ec.message() << "\n";
            rcFreeHeightField(solid);
            rcFreeHeightField(liquid);
            return false;
        }

        bool const saved = tile.Save(outputFile.string());
        rcFreeHeightField(solid);
        rcFreeHeightField(liquid);

        if (!saved)
        {
            std::cerr << "failed to write " << outputFile << "\n";
            return false;
        }

        std::cout << "wrote " << outputFile << " (" << waterColumns << " water columns, " << waterCells
                  << " cells, " << tile.Intervals().size() << " intervals)\n";
        return true;
    }
}

int main(int argc, char** argv)
{
    Options options;
    if (!ParseOptions(argc, argv, options))
    {
        PrintUsage(argv[0]);
        return 1;
    }

    if (options.CellSize <= 0.0f)
    {
        std::cerr << "cell size must be positive\n";
        return 1;
    }

    if (options.Tiles.empty())
    {
        if (!options.HasCenter)
        {
            std::cerr << "no tiles requested; use --tile or --center-x/--center-y\n";
            return 1;
        }

        uint32 const centerX = TileFromWorld(options.CenterX);
        uint32 const centerY = TileFromWorld(options.CenterY);
        uint32 const radiusTiles = uint32(std::ceil(options.Radius / MMAP::GRID_SIZE));
        for (uint32 x = centerX >= radiusTiles ? centerX - radiusTiles : 0;
             x <= centerX + radiusTiles && x < 64; ++x)
        {
            for (uint32 y = centerY >= radiusTiles ? centerY - radiusTiles : 0;
                 y <= centerY + radiusTiles && y < 64; ++y)
                options.Tiles.emplace(x, y);
        }
    }

    if (options.Tiles.empty())
    {
        std::cerr << "no tiles to generate\n";
        return 1;
    }

    std::cout << "nav3d_generator map " << options.MapId << " cell " << options.CellSize << " tiles "
              << options.Tiles.size() << " output " << options.OutputDir << "\n";

    for (auto const& tile : options.Tiles)
    {
        if (!GenerateTile(options, tile.first, tile.second))
            return 1;
    }

    return 0;
}
