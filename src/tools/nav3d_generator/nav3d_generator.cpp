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
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace
{
    constexpr float kDefaultCellSize = 1.0f;
    constexpr float kMinCellSize = 0.5f;
    constexpr float kMaxCellSize = 4.0f;
    constexpr uint32 kGeneratorVersion = 3;
    constexpr size_t kMaxVoxelCells = 512ull * 1024ull * 1024ull;
    constexpr float kBodyRadius = 0.45f;
    constexpr float kBodyDiagonal = kBodyRadius * 1.41421356f;
    constexpr float kBodyHalfHeight = 0.8f;

    struct TileRequest
    {
        uint32 MapId = 0;
        uint32 TileX = 0;
        uint32 TileY = 0;
    };

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
        bool TerrainOnly = false;
        std::set<std::pair<uint32, uint32>> Tiles;
        std::vector<TileRequest> Requests;
    };

    void PrintUsage(char const* program)
    {
        std::cout << "Usage: " << program << " --data-dir <dir> [options]\n"
                  << "  --output <dir>       Output directory (default: <data-dir>/nav3d)\n"
                  << "  --map <id>           Map id (default: 0)\n"
                  << "  --cell-size <yd>     Grid cell size (default: 1.0)\n"
                  << "  --tile <x:y>         Tile coordinate; repeatable\n"
                  << "  --tiles-file <path>  Lines of '<map> <x> <y>' or '<x> <y>'\n"
                  << "  --center-x <x>       Generate tiles around this world X\n"
                  << "  --center-y <y>       Generate tiles around this world Y\n"
                  << "  --radius <yd>        Radius around the center (default: 0)\n"
                  << "  --water-level <z>    Fill water up to this Z (default: local liquid surface)\n"
                  << "  --min-z <z>          Lowest Z included in the tile (default: mesh min)\n"
                  << "  --terrain-only       Read .map terrain/liquid directly (offline water; no WMO)\n";
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
            else if (arg == "--tiles-file")
            {
                if (!next(value))
                    return false;
                std::ifstream in(value);
                if (!in)
                {
                    std::cerr << "cannot open tiles file: " << value << "\n";
                    return false;
                }
                std::string line;
                while (std::getline(in, line))
                {
                    if (line.empty() || line[0] == '#')
                        continue;
                    for (char& c : line)
                        if (c == ',')
                            c = ' ';
                    std::istringstream row(line);
                    TileRequest request;
                    uint32 first = 0;
                    uint32 second = 0;
                    uint32 third = 0;
                    if (row >> first >> second >> third)
                    {
                        request.MapId = first;
                        request.TileX = second;
                        request.TileY = third;
                    }
                    else
                    {
                        row.clear();
                        row.str(line);
                        if (!(row >> first >> second))
                        {
                            std::cerr << "bad tiles-file line: " << line << "\n";
                            return false;
                        }
                        request.MapId = options.MapId;
                        request.TileX = first;
                        request.TileY = second;
                    }
                    if (request.TileX >= 64 || request.TileY >= 64)
                    {
                        std::cerr << "tile out of range: " << line << "\n";
                        return false;
                    }
                    options.Requests.push_back(request);
                }
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
            else if (arg == "--terrain-only")
            {
                options.TerrainOnly = true;
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
        // TerrainBuilder names its map file <tileY><tileX>.map. The server grid uses
        // <tileX><tileY>.map, so load with the swapped pair and shift the heightmap/liquid
        // vertices back into this tile's server-order frame.
        size_t const solidStart = meshData.solidVerts.size();
        size_t const liquidStart = meshData.liquidVerts.size();
        terrainBuilder.loadMap(options.MapId, tileY, tileX, meshData);
        for (size_t i = solidStart; i + 2 < meshData.solidVerts.size(); i += 3)
        {
            meshData.solidVerts[i] -= MMAP::GRID_SIZE;
            meshData.solidVerts[i + 2] += MMAP::GRID_SIZE;
        }
        for (size_t i = liquidStart; i + 2 < meshData.liquidVerts.size(); i += 3)
        {
            meshData.liquidVerts[i] -= MMAP::GRID_SIZE;
            meshData.liquidVerts[i + 2] += MMAP::GRID_SIZE;
        }
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

        // Body-box solid pass. The movement body is an axis-aligned box; a water cell is
        // navigable only if no solid span lies within the box footprint and vertical body
        // band. This is the offline equivalent of the live Map LOS bake.
        {
            int const range = std::max(1, int(std::ceil((kBodyDiagonal + cellSize * 0.5f) / cellSize)));
            int const verticalCells = std::max(1, int(std::round(kBodyHalfHeight / cellSize)));
            size_t blockedCells = 0;
            for (int z = 0; z < sizeZ; ++z)
            {
                for (int y = 0; y < height; ++y)
                {
                    for (int x = 0; x < width; ++x)
                    {
                        size_t const index = cellIndex(x, y, z);
                        if (!water[index])
                            continue;

                        bool blocked = false;
                        for (int dy = -range; dy <= range && !blocked; ++dy)
                        {
                            for (int dx = -range; dx <= range && !blocked; ++dx)
                            {
                                if (dx == 0 && dy == 0)
                                    continue;

                                float const distance =
                                    std::sqrt(float(dx * dx + dy * dy)) * cellSize;
                                float const limit =
                                    (dx == 0 || dy == 0) ? kBodyRadius : kBodyDiagonal;
                                if (distance > limit + cellSize * 0.5f)
                                    continue;

                                int const nx = x + dx;
                                int const ny = y + dy;
                                if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                                    continue;

                                rcSpan const* spans = solid->spans[ny * width + nx];
                                if (!spans)
                                    continue;

                                int const lowZ = std::max(0, z - verticalCells);
                                int const highZ = std::min(sizeZ - 1, z + verticalCells);
                                for (int zz = lowZ; zz <= highZ; ++zz)
                                {
                                    if (SolidAt(spans, zz))
                                    {
                                        blocked = true;
                                        break;
                                    }
                                }
                            }
                        }

                        if (blocked)
                        {
                            water[index] = 0;
                            ++blockedCells;
                        }
                    }
                }
            }
            std::cout << "solid pass removed " << blockedCells << " cells\n";
        }

        // Horizontal clearance: distance in the XY plane to the nearest non-water cell in
        // the same Z slice. The water surface and floor must not shrink it.
        auto const relax2D = [&distance, width, height](uint8& value, int x, int y, int z)
        {
            if (x < 0 || y < 0 || x >= width || y >= height)
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
                    distance[index] = water[index] ? 255 : 0;
                }
            }

            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                        continue;
                    uint8 value = 255;
                    relax2D(value, x - 1, y, z);
                    relax2D(value, x, y - 1, z);
                    distance[index] = value;
                }
            }

            for (int y = height - 1; y >= 0; --y)
            {
                for (int x = width - 1; x >= 0; --x)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                        continue;
                    uint8 value = distance[index];
                    relax2D(value, x + 1, y, z);
                    relax2D(value, x, y + 1, z);
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
                uint8 currentClearance = 0;

                auto const flush = [&](int end)
                {
                    if (start < 0)
                        return;
                    Nav3DTileInterval interval;
                    interval.ZMin = int16(start);
                    interval.ZMax = int16(end);
                    interval.Clearance = currentClearance;
                    interval.Flags = 0;
                    if (column.IntervalCount == 0)
                        column.IntervalOffset = uint32(tile.Intervals().size());
                    column.IntervalCount = uint16(column.IntervalCount + 1);
                    tile.Intervals().push_back(interval);
                    start = -1;
                    currentClearance = 0;
                };

                for (int z = 0; z < sizeZ; ++z)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                    {
                        flush(z - 1);
                        continue;
                    }

                    uint32 const rawClearance = std::min<uint32>(
                        16, uint32(float(distance[index]) * cellSize / Nav3D::ClearanceUnit));
                    uint8 const clearance = uint8((rawClearance / 2) * 2);
                    if (start < 0)
                    {
                        start = z;
                        currentClearance = clearance;
                    }
                    else if (clearance != currentClearance)
                    {
                        flush(z - 1);
                        start = z;
                        currentClearance = clearance;
                    }
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

    struct TerrainMapFile
    {
        bool HasHeight = false;
        bool HasLiquid = false;
        float GridHeight = 0.0f;
        float HeightMultiplier = 0.0f;
        std::vector<float> V9;
        std::vector<float> V8;
        std::array<uint16, 256> Holes{};
        bool HasHoles = false;
        float LiquidLevel = -1000.0f;
        uint8 LiquidOffX = 0;
        uint8 LiquidOffY = 0;
        uint8 LiquidWidth = 0;
        uint8 LiquidHeight = 0;
        std::vector<float> LiquidMap;
    };

    bool LoadTerrainMap(Options const& options, uint32 tileX, uint32 tileY, TerrainMapFile& mapData)
    {
        std::string const path = Acore::StringFormat("{}/maps/{:03}{:02}{:02}.map", options.DataDir,
                                                     options.MapId, tileX, tileY);
        std::ifstream in(path, std::ios::binary);
        if (!in)
            return false;

        uint32 header[11]{};
        in.read(reinterpret_cast<char*>(header), sizeof(header));
        if (!in)
            return false;

        if (header[5])
        {
            in.seekg(header[5]);
            uint32 fourcc = 0;
            uint32 flags = 0;
            float gridHeight = 0.0f;
            float gridMaxHeight = 0.0f;
            in.read(reinterpret_cast<char*>(&fourcc), sizeof(fourcc));
            in.read(reinterpret_cast<char*>(&flags), sizeof(flags));
            in.read(reinterpret_cast<char*>(&gridHeight), sizeof(gridHeight));
            in.read(reinterpret_cast<char*>(&gridMaxHeight), sizeof(gridMaxHeight));
            if (!in)
                return false;

            mapData.GridHeight = gridHeight;
            if (!(flags & 0x0001))
            {
                if (flags & 0x0002)
                {
                    std::vector<uint16> v9(129 * 129);
                    std::vector<uint16> v8(128 * 128);
                    in.read(reinterpret_cast<char*>(v9.data()), v9.size() * sizeof(uint16));
                    in.read(reinterpret_cast<char*>(v8.data()), v8.size() * sizeof(uint16));
                    if (!in)
                        return false;
                    mapData.HeightMultiplier = (gridMaxHeight - gridHeight) / 65535.0f;
                    mapData.V9.resize(v9.size());
                    mapData.V8.resize(v8.size());
                    for (size_t i = 0; i < v9.size(); ++i)
                        mapData.V9[i] = float(v9[i]) * mapData.HeightMultiplier + gridHeight;
                    for (size_t i = 0; i < v8.size(); ++i)
                        mapData.V8[i] = float(v8[i]) * mapData.HeightMultiplier + gridHeight;
                }
                else if (flags & 0x0004)
                {
                    std::vector<uint8> v9(129 * 129);
                    std::vector<uint8> v8(128 * 128);
                    in.read(reinterpret_cast<char*>(v9.data()), v9.size() * sizeof(uint8));
                    in.read(reinterpret_cast<char*>(v8.data()), v8.size() * sizeof(uint8));
                    if (!in)
                        return false;
                    mapData.HeightMultiplier = (gridMaxHeight - gridHeight) / 255.0f;
                    mapData.V9.resize(v9.size());
                    mapData.V8.resize(v8.size());
                    for (size_t i = 0; i < v9.size(); ++i)
                        mapData.V9[i] = float(v9[i]) * mapData.HeightMultiplier + gridHeight;
                    for (size_t i = 0; i < v8.size(); ++i)
                        mapData.V8[i] = float(v8[i]) * mapData.HeightMultiplier + gridHeight;
                }
                else
                {
                    mapData.V9.resize(129 * 129);
                    mapData.V8.resize(128 * 128);
                    in.read(reinterpret_cast<char*>(mapData.V9.data()), mapData.V9.size() * sizeof(float));
                    in.read(reinterpret_cast<char*>(mapData.V8.data()), mapData.V8.size() * sizeof(float));
                    if (!in)
                        return false;
                    mapData.HeightMultiplier = 1.0f;
                }
                mapData.HasHeight = true;
            }
        }

        if (header[9] && header[10] >= sizeof(mapData.Holes))
        {
            in.seekg(header[9]);
            in.read(reinterpret_cast<char*>(mapData.Holes.data()), sizeof(mapData.Holes));
            mapData.HasHoles = in.good();
        }

        if (header[7])
        {
            in.seekg(header[7]);
            uint32 fourcc = 0;
            uint8 flags = 0;
            uint8 liquidFlags = 0;
            uint16 liquidType = 0;
            uint8 offX = 0;
            uint8 offY = 0;
            uint8 width = 0;
            uint8 height = 0;
            float level = -1000.0f;
            in.read(reinterpret_cast<char*>(&fourcc), sizeof(fourcc));
            in.read(reinterpret_cast<char*>(&flags), sizeof(flags));
            in.read(reinterpret_cast<char*>(&liquidFlags), sizeof(liquidFlags));
            in.read(reinterpret_cast<char*>(&liquidType), sizeof(liquidType));
            in.read(reinterpret_cast<char*>(&offX), sizeof(offX));
            in.read(reinterpret_cast<char*>(&offY), sizeof(offY));
            in.read(reinterpret_cast<char*>(&width), sizeof(width));
            in.read(reinterpret_cast<char*>(&height), sizeof(height));
            in.read(reinterpret_cast<char*>(&level), sizeof(level));
            if (!in)
                return false;

            mapData.LiquidOffX = offX;
            mapData.LiquidOffY = offY;
            mapData.LiquidWidth = width;
            mapData.LiquidHeight = height;
            mapData.LiquidLevel = level;
            mapData.HasLiquid = true;

            if (!(flags & 0x0001))
            {
                in.seekg(512 + 256, std::ios::cur);
                if (!in)
                    return false;
            }
            if (!(flags & 0x0002))
            {
                mapData.LiquidMap.resize(size_t(width) * size_t(height));
                in.read(reinterpret_cast<char*>(mapData.LiquidMap.data()),
                        mapData.LiquidMap.size() * sizeof(float));
                if (!in)
                    return false;
            }
        }

        return true;
    }

    bool IsTerrainHole(TerrainMapFile const& mapData, int x, int y)
    {
        if (!mapData.HasHoles)
            return false;

        static constexpr uint16 holeH[4] = {0x1111, 0x2222, 0x4444, 0x8888};
        static constexpr uint16 holeV[4] = {0x000F, 0x00F0, 0x0F00, 0xF000};
        int const cellRow = x / 8;
        int const cellCol = y / 8;
        int const holeRow = (x % 8) / 2;
        int const holeCol = (y % 8) / 2;
        uint16 const hole = mapData.Holes[cellRow * 16 + cellCol];
        return (hole & holeH[holeCol] & holeV[holeRow]) != 0;
    }

    float TerrainHeight(TerrainMapFile const& mapData, float wx, float wy)
    {
        if (!mapData.HasHeight)
            return -1000.0f;

        float x = 128.0f * (32.0f - wx / 533.3333f);
        float y = 128.0f * (32.0f - wy / 533.3333f);
        int xInt = int(x);
        int yInt = int(y);
        x -= float(xInt);
        y -= float(yInt);
        xInt &= 127;
        yInt &= 127;

        if (IsTerrainHole(mapData, xInt, yInt))
            return -1000.0f;

        float const h1 = mapData.V9[xInt * 129 + yInt];
        float const h2 = mapData.V9[(xInt + 1) * 129 + yInt];
        float const h3 = mapData.V9[xInt * 129 + yInt + 1];
        float const h4 = mapData.V9[(xInt + 1) * 129 + yInt + 1];
        float const h5 = 2.0f * mapData.V8[xInt * 128 + yInt] - mapData.GridHeight;

        float a = 0.0f;
        float b = 0.0f;
        float c = 0.0f;
        if (x + y < 1.0f)
        {
            if (x > y)
            {
                a = h2 - h1;
                b = h5 - h1 - h2;
                c = h1;
            }
            else
            {
                a = h5 - h1 - h3;
                b = h3 - h1;
                c = h1;
            }
        }
        else
        {
            if (x > y)
            {
                a = h2 + h4 - h5;
                b = h4 - h2;
                c = h5 - h4;
            }
            else
            {
                a = h4 - h3;
                b = h3 + h4 - h5;
                c = h5 - h4;
            }
        }

        return a * x + b * y + c;
    }

    float TerrainLiquidLevel(TerrainMapFile const& mapData, float wx, float wy)
    {
        if (!mapData.HasLiquid)
            return -1000.0f;

        if (mapData.LiquidMap.empty())
            return mapData.LiquidLevel;

        float x = 128.0f * (32.0f - wx / 533.3333f);
        float y = 128.0f * (32.0f - wy / 533.3333f);
        int const cx = (int(x) & 127) - mapData.LiquidOffY;
        int const cy = (int(y) & 127) - mapData.LiquidOffX;
        if (cx < 0 || cx >= mapData.LiquidHeight || cy < 0 || cy >= mapData.LiquidWidth)
            return -1000.0f;
        return mapData.LiquidMap[size_t(cx) * size_t(mapData.LiquidWidth) + size_t(cy)];
    }

    bool GenerateTerrainWaterTile(Options const& options, uint32 tileX, uint32 tileY)
    {
        TerrainMapFile mapData;
        if (!LoadTerrainMap(options, tileX, tileY, mapData))
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (no readable .map)\n";
            return true;
        }

        float const cellSize = options.CellSize;
        float const originX = (32.0f - float(tileX)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        float const originY = (32.0f - float(tileY)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        int const width = int(std::ceil(MMAP::GRID_SIZE / cellSize));
        int const height = int(std::ceil(MMAP::GRID_SIZE / cellSize));

        float minTerrain = std::numeric_limits<float>::max();
        for (float heightValue : mapData.V9)
            minTerrain = std::min(minTerrain, heightValue);

        float maxLiquid = -std::numeric_limits<float>::max();
        if (mapData.LiquidLevel > -1000.0f)
            maxLiquid = mapData.LiquidLevel;
        for (float liquidValue : mapData.LiquidMap)
            if (liquidValue > -1000.0f)
                maxLiquid = std::max(maxLiquid, liquidValue);

        if (maxLiquid <= -1000.0f && !options.HasWaterLevel)
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (no liquid)\n";
            return true;
        }

        float const minZ = options.HasMinZ ? options.MinZ : minTerrain - cellSize;
        float const maxZ = options.HasWaterLevel ? options.WaterLevel + cellSize : maxLiquid + cellSize;
        if (maxZ <= minZ)
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (no water z range)\n";
            return true;
        }
        int const sizeZ = std::max(1, int(std::ceil((maxZ - minZ) / cellSize)));
        size_t const cellCount = size_t(width) * size_t(height) * size_t(sizeZ);
        if (sizeZ > 32767 || cellCount > kMaxVoxelCells)
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (z range too large: " << sizeZ << ", cells " << cellCount << ")\n";
            return true;
        }

        auto const cellIndex = [width, height, sizeZ](int x, int y, int z) -> size_t
        {
            return (size_t(z) * size_t(height) + size_t(y)) * size_t(width) + size_t(x);
        };

        struct Column
        {
            float Floor = -1000.0f;
            float WaterTop = -1000.0f;
        };
        std::vector<Column> columns(size_t(width) * size_t(height));
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                float const wx = originX + (float(x) + 0.5f) * cellSize;
                float const wy = originY + (float(y) + 0.5f) * cellSize;
                Column& column = columns[size_t(y) * size_t(width) + size_t(x)];
                column.Floor = TerrainHeight(mapData, wx, wy);
                column.WaterTop = options.HasWaterLevel ? options.WaterLevel : TerrainLiquidLevel(mapData, wx, wy);
            }
        }

        std::vector<uint8> water(cellCount, 0);
        size_t waterColumns = 0;
        size_t waterCells = 0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                Column const& column = columns[size_t(y) * size_t(width) + size_t(x)];
                if (column.Floor <= -1000.0f || column.WaterTop <= -1000.0f ||
                    column.WaterTop <= column.Floor + 0.05f)
                    continue;

                int const firstZ = std::clamp(int(std::ceil((column.Floor - minZ) / cellSize)), 0, sizeZ);
                int const lastZ = std::clamp(int(std::floor((column.WaterTop - minZ) / cellSize)), 0, sizeZ - 1);
                bool any = false;
                for (int z = firstZ; z <= lastZ; ++z)
                {
                    float const worldZ = minZ + (float(z) + 0.5f) * cellSize;
                    if (worldZ <= column.Floor || worldZ > column.WaterTop)
                        continue;
                    water[cellIndex(x, y, z)] = 1;
                    ++waterCells;
                    any = true;
                }
                if (any)
                    ++waterColumns;
            }
        }

        int const range = std::max(1, int(std::ceil((kBodyDiagonal + cellSize * 0.5f) / cellSize)));
        size_t blockedCells = 0;
        for (int z = 0; z < sizeZ; ++z)
        {
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                        continue;

                    float const worldZ = minZ + (float(z) + 0.5f) * cellSize;
                    bool blocked = false;
                    for (int dy = -range; dy <= range && !blocked; ++dy)
                    {
                        for (int dx = -range; dx <= range && !blocked; ++dx)
                        {
                            if (dx == 0 && dy == 0)
                                continue;
                            float const dist = std::sqrt(float(dx * dx + dy * dy)) * cellSize;
                            float const limit = (dx == 0 || dy == 0) ? kBodyRadius : kBodyDiagonal;
                            if (dist > limit + cellSize * 0.5f)
                                continue;
                            int const nx = x + dx;
                            int const ny = y + dy;
                            if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                                continue;
                            Column const& neighbor = columns[size_t(ny) * size_t(width) + size_t(nx)];
                            if (neighbor.Floor <= -1000.0f)
                                continue;
                            if (neighbor.Floor >= worldZ - 0.05f)
                            {
                                blocked = true;
                                break;
                            }
                        }
                    }
                    if (blocked)
                    {
                        water[index] = 0;
                        ++blockedCells;
                    }
                }
            }
        }

        std::vector<uint8> distance(cellCount, 0);
        auto const relax2D = [&distance, width, height](uint8& value, int x, int y, int z)
        {
            if (x < 0 || y < 0 || x >= width || y >= height)
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
                for (int x = 0; x < width; ++x)
                    distance[cellIndex(x, y, z)] = water[cellIndex(x, y, z)] ? 255 : 0;

            for (int y = 0; y < height; ++y)
                for (int x = 0; x < width; ++x)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                        continue;
                    uint8 value = 255;
                    relax2D(value, x - 1, y, z);
                    relax2D(value, x, y - 1, z);
                    distance[index] = value;
                }

            for (int y = height - 1; y >= 0; --y)
                for (int x = width - 1; x >= 0; --x)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                        continue;
                    uint8 value = distance[index];
                    relax2D(value, x + 1, y, z);
                    relax2D(value, x, y + 1, z);
                    distance[index] = value;
                }
        }

        Nav3DTile tile;
        Nav3DTileHeader& header = tile.Header();
        header.MapId = options.MapId;
        header.TileX = uint16(tileX);
        header.TileY = uint16(tileY);
        header.Medium = Nav3D::MediumWater;
        header.CellSize = cellSize;
        header.OriginX = originX;
        header.OriginY = originY;
        header.OriginZ = minZ;
        header.ExtentX = float(width) * cellSize;
        header.ExtentY = float(height) * cellSize;
        header.ExtentZ = float(sizeZ) * cellSize;
        header.SizeX = uint32(width);
        header.SizeY = uint32(height);
        header.SizeZ = uint32(sizeZ);
        header.Flags |= Nav3D::FlagHasSolid | Nav3D::FlagLiveLos;
        header.GeneratorVersion = kGeneratorVersion;
        header.GeometryHash = 0;

        tile.Columns().resize(size_t(width) * size_t(height));
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                Nav3DTileColumn& column = tile.Columns()[size_t(y) * size_t(width) + size_t(x)];
                int start = -1;
                uint8 currentClearance = 0;
                auto const flush = [&](int end)
                {
                    if (start < 0)
                        return;
                    Nav3DTileInterval interval;
                    interval.ZMin = int16(start);
                    interval.ZMax = int16(end);
                    interval.Clearance = currentClearance;
                    interval.Flags = 0;
                    if (column.IntervalCount == 0)
                        column.IntervalOffset = uint32(tile.Intervals().size());
                    column.IntervalCount = uint16(column.IntervalCount + 1);
                    tile.Intervals().push_back(interval);
                    start = -1;
                    currentClearance = 0;
                };

                for (int z = 0; z < sizeZ; ++z)
                {
                    size_t const index = cellIndex(x, y, z);
                    if (!water[index])
                    {
                        flush(z - 1);
                        continue;
                    }
                    uint32 const rawClearance = std::min<uint32>(
                        16, uint32(float(distance[index]) * cellSize / Nav3D::ClearanceUnit));
                    uint8 const clearance = uint8((rawClearance / 2) * 2);
                    if (start < 0)
                    {
                        start = z;
                        currentClearance = clearance;
                    }
                    else if (clearance != currentClearance)
                    {
                        flush(z - 1);
                        start = z;
                        currentClearance = clearance;
                    }
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
            return false;
        }

        if (!tile.Save(outputFile.string()))
        {
            std::cerr << "failed to write " << outputFile << "\n";
            return false;
        }

        std::cout << "wrote " << outputFile << " (" << waterColumns << " water columns, " << waterCells
                  << " cells, " << tile.Intervals().size() << " intervals, " << blockedCells
                  << " box-blocked)\n";
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

    std::vector<TileRequest> requests = options.Requests;
    for (auto const& tile : options.Tiles)
        requests.push_back({options.MapId, tile.first, tile.second});

    if (requests.empty() && options.HasCenter)
    {
        uint32 const centerX = TileFromWorld(options.CenterX);
        uint32 const centerY = TileFromWorld(options.CenterY);
        uint32 const radiusTiles = uint32(std::ceil(options.Radius / MMAP::GRID_SIZE));
        for (uint32 x = centerX >= radiusTiles ? centerX - radiusTiles : 0;
             x <= centerX + radiusTiles && x < 64; ++x)
        {
            for (uint32 y = centerY >= radiusTiles ? centerY - radiusTiles : 0;
                 y <= centerY + radiusTiles && y < 64; ++y)
                requests.push_back({options.MapId, x, y});
        }
    }

    if (requests.empty())
    {
        std::cerr << "no tiles to generate\n";
        return 1;
    }

    std::cout << "nav3d_generator cell " << options.CellSize << " tiles " << requests.size()
              << " output " << options.OutputDir << "\n";

    size_t failed = 0;
    for (TileRequest const& request : requests)
    {
        Options tileOptions = options;
        tileOptions.MapId = request.MapId;
        bool const generated = tileOptions.TerrainOnly
                                   ? GenerateTerrainWaterTile(tileOptions, request.TileX, request.TileY)
                                   : GenerateTile(tileOptions, request.TileX, request.TileY);
        if (!generated)
        {
            std::cerr << "failed map " << request.MapId << " tile " << request.TileX << ":" << request.TileY
                      << "\n";
            ++failed;
        }
    }

    return failed == 0 ? 0 : 1;
}
