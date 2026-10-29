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
#include "MapTree.h"
#include "MapDefines.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "ModelInstance.h"
#include "StringFormat.h"
#include "VMapMgr2.h"
#include "mmaps_generator/TerrainBuilder.h"

#include "Recast.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <thread>
#include <atomic>
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
        bool HasMaxZ = false;
        float MaxZ = 0.0f;
        bool HasMaxDepth = false;
        float MaxDepth = 200.0f;
        bool TerrainOnly = false;
        bool Air = false;
        bool AirVolume = false;
        bool Probe = false;
        float ProbeX = 0.0f;
        float ProbeY = 0.0f;
        float ProbeZ = 0.0f;
        bool AirTreeDetail = false;
        bool AirDetailDiag = false;
        bool SkipExisting = false;
        std::string ManifestPath;
        bool HasBox = false;
        float BoxMinX = 0.0f;
        float BoxMaxX = 0.0f;
        float BoxMinY = 0.0f;
        float BoxMaxY = 0.0f;
        float BoxMinZ = 0.0f;
        float BoxMaxZ = 0.0f;
        std::vector<std::array<float, 3>> Seeds;
        uint32 Jobs = 1;
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
                  << "  --max-z <z>          Highest Z included in the tile (default: mesh max)\n"
                  << "  --max-depth <yd>     Terrain-water depth below the liquid surface (default: full)\n"
                  << "  --terrain-only       Read .map terrain/liquid directly (offline water; no WMO)\n"
                  << "  --air                Write a terrain air-heightmap tile (_air.n3t)\n"
                  << "  --air-volume         Write a volumetric air tile with WMO spans\n"
                  << "  --probe <x> <y> <z>  Dump VMAP model/height probes at a world point\n"
                  << "  --air-tree-detail    Bake a 1-yd air detail box from StaticMapTree + .map\n"
                  << "  --box minX maxX minY maxY minZ maxZ\n"
                  << "  --seed x y z         Flood-fill seed for --air-tree-detail; repeatable\n"
                  << "  --air-detail-diag   Print per-gate classification for every --seed, then exit\n"
                  << "  --skip-existing      Skip a tile whose output file already exists\n"
                  << "  --manifest <path>    Append per-tile results (map tileX tileY status)\n"
                  << "  --jobs <n>           Parallel classification workers (default 1)\n";
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
            else if (arg == "--max-z")
            {
                if (!next(value) || !ParseFloat(value, options.MaxZ))
                    return false;
                options.HasMaxZ = true;
            }
            else if (arg == "--max-depth")
            {
                if (!next(value) || !ParseFloat(value, options.MaxDepth) || options.MaxDepth <= 0.0f)
                    return false;
                options.HasMaxDepth = true;
            }
            else if (arg == "--terrain-only")
            {
                options.TerrainOnly = true;
            }
            else if (arg == "--air")
            {
                options.Air = true;
            }
            else if (arg == "--air-volume")
            {
                options.AirVolume = true;
            }
            else if (arg == "--probe")
            {
                options.Probe = true;
                if (i + 3 >= argc || !ParseFloat(argv[i + 1], options.ProbeX) ||
                    !ParseFloat(argv[i + 2], options.ProbeY) ||
                    !ParseFloat(argv[i + 3], options.ProbeZ))
                    return false;
                i += 3;
            }
            else if (arg == "--air-tree-detail")
            {
                options.AirTreeDetail = true;
            }
            else if (arg == "--air-detail-diag")
            {
                options.AirDetailDiag = true;
            }
            else if (arg == "--skip-existing")
            {
                options.SkipExisting = true;
            }
            else if (arg == "--manifest")
            {
                if (!next(options.ManifestPath))
                    return false;
            }
            else if (arg == "--box")
            {
                if (i + 6 >= argc || !ParseFloat(argv[i + 1], options.BoxMinX) ||
                    !ParseFloat(argv[i + 2], options.BoxMaxX) ||
                    !ParseFloat(argv[i + 3], options.BoxMinY) ||
                    !ParseFloat(argv[i + 4], options.BoxMaxY) ||
                    !ParseFloat(argv[i + 5], options.BoxMinZ) ||
                    !ParseFloat(argv[i + 6], options.BoxMaxZ))
                    return false;
                options.HasBox = true;
                i += 6;
            }
            else if (arg == "--seed")
            {
                std::array<float, 3> seed{};
                if (i + 3 >= argc || !ParseFloat(argv[i + 1], seed[0]) ||
                    !ParseFloat(argv[i + 2], seed[1]) ||
                    !ParseFloat(argv[i + 3], seed[2]))
                    return false;
                options.Seeds.push_back(seed);
                i += 3;
            }
            else if (arg == "--jobs")
            {
                uint32 jobs = 1;
                if (!next(value) || !ParseUInt(value, jobs) || jobs == 0)
                    return false;
                options.Jobs = jobs;
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

        // TerrainBuilder::copyVertices stores vertices as (world y, world z, world x).
        auto const include = [&bounds, &any](G3D::Array<float> const& verts)
        {
            for (int i = 0; i + 2 < verts.size(); i += 3)
            {
                bounds.MinX = std::min(bounds.MinX, verts[i + 2]);
                bounds.MinY = std::min(bounds.MinY, verts[i]);
                bounds.MinZ = std::min(bounds.MinZ, verts[i + 1]);
                bounds.MaxX = std::max(bounds.MaxX, verts[i + 2]);
                bounds.MaxY = std::max(bounds.MaxY, verts[i]);
                bounds.MaxZ = std::max(bounds.MaxZ, verts[i + 1]);
                any = true;
            }
        };

        include(mesh.solidVerts);
        include(mesh.liquidVerts);
        return any;
    }

    // Non-tiled maps keep their single global WMO in the .vmtree GOBJ chunk. Loading it
    // once per grid cell would duplicate the whole model nine times in the 3x3 loop.
    bool VMapIsTiled(std::string const& dataDir, uint32 mapId)
    {
        std::filesystem::path const path = std::filesystem::path(dataDir) / "vmaps" /
            Acore::StringFormat("{:03}.vmtree", mapId);
        std::ifstream in(path, std::ios::binary);
        char magic[8]{};
        char tiled = 1;
        if (!in.read(magic, sizeof(magic)) || !in.read(&tiled, 1))
            return true;
        return tiled != 0;
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

    // Terrain .map entries carry MAP_LIQUID_TYPE_* with bit 31 set. WMO entries
    // carry the LiquidType.dbc id; only water/ocean ids are swimmable.
    bool IsWaterLiquid(uint32 rawType)
    {
        if (rawType & 0x80000000u)
        {
            uint32 const flags = rawType & 0x7fffffffu;
            return (flags & (0x01u | 0x02u)) != 0 && (flags & 0x10u) == 0;
        }

        switch (rawType)
        {
            case 1:   // Water
            case 2:   // Ocean
            case 5:   // Slow Water
            case 6:   // Slow Ocean
            case 9:   // Fast Water
            case 10:  // Fast Ocean
            case 13:  // WMO Water
            case 14:  // WMO Ocean
            case 17:  // WMO Water - Interior
            case 41:  // Coilfang Raid - Water
            case 61:  // Hyjal Past - Water
            case 81:  // Lake Wintergrasp - Water
            case 100: // Basic Procedural Water
            case 181: // Orange Slime (water movement type)
                return true;
            default:
                return false;
        }
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
        bool const vmapTiled = VMapIsTiled(options.DataDir, options.MapId);
        for (int offsetX = -1; offsetX <= 1; ++offsetX)
        {
            for (int offsetY = -1; offsetY <= 1; ++offsetY)
            {
                if (!vmapTiled && (offsetX != 0 || offsetY != 0))
                    continue;
                int const wmoX = int(tileX) + offsetX;
                int const wmoY = int(tileY) + offsetY;
                if (wmoX < 0 || wmoY < 0 || wmoX >= 64 || wmoY >= 64)
                    continue;
                terrainBuilder.loadVMap(options.MapId, uint32(wmoX), uint32(wmoY), meshData);
            }
        }
        MMAP::TerrainBuilder::cleanVertices(meshData.solidVerts, meshData.solidTris);
        MMAP::TerrainBuilder::cleanVertices(meshData.liquidVerts, meshData.liquidTris);

        // TerrainBuilder also emits magma and slime surfaces. Only water is swimmable;
        // keep the water triangles and leave the other liquid types out of the volume.
        G3D::Array<int> waterLiquidTris;
        for (size_t i = 0; i + 2 < meshData.liquidTris.size(); i += 3)
        {
            size_t const triangle = i / 3;
            bool const isWater =
                triangle < meshData.liquidTypeRaw.size()
                    ? IsWaterLiquid(meshData.liquidTypeRaw[triangle])
                    : (triangle < meshData.liquidType.size() &&
                       meshData.liquidType[triangle] == NAV_WATER);
            if (!isWater)
                continue;
            waterLiquidTris.append(meshData.liquidTris[i], meshData.liquidTris[i + 1],
                                   meshData.liquidTris[i + 2]);
        }

        if (meshData.solidVerts.size() == 0 || meshData.solidTris.size() == 0 ||
            (!options.HasWaterLevel && waterLiquidTris.size() == 0))
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

        float const minZ = options.HasMinZ ? options.MinZ : meshBounds.MinZ - cellSize;
        float const globalTop = options.HasWaterLevel
                                    ? options.WaterLevel
                                    : (options.HasMaxZ ? options.MaxZ : meshBounds.MaxZ + cellSize);
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

        // TerrainBuilder::copyVertices writes Recast vertices as (world y, world z,
        // world x); the Recast bounds must use the same axis order.
        float bmin[3] = {recastMinZ, minZ, recastMinX};
        float bmax[3] = {recastMaxZ, minZ + float(sizeZ) * cellSize, recastMaxX};

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
        int const liquidTriangles = waterLiquidTris.size() / 3;
        std::vector<unsigned char> solidAreas(solidTriangles, RC_WALKABLE_AREA);
        std::vector<unsigned char> liquidAreas(liquidTriangles, RC_WALKABLE_AREA);

        if (!rcRasterizeTriangles(&context, meshData.solidVerts.getCArray(), meshData.solidVerts.size() / 3,
                                  meshData.solidTris.getCArray(), solidAreas.data(), solidTriangles, *solid) ||
            !rcRasterizeTriangles(&context, meshData.liquidVerts.getCArray(), meshData.liquidVerts.size() / 3,
                                  waterLiquidTris.getCArray(), liquidAreas.data(), liquidTriangles, *liquid))
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

                float const localTop = LiquidTop(liquid, column, bmin[1], cellSize);
                float const waterTop = options.HasWaterLevel ? options.WaterLevel : localTop;
                if (waterTop <= bmin[1])
                    continue;

                int const waterTopCell =
                    std::clamp(int(std::floor((waterTop - bmin[1]) / cellSize)), 0, sizeZ - 1);

                // Water is the complement of solid spans up to the water surface. Below the
                // lowest solid span (seabed/hull bottom) is solid/void, not water.
                std::vector<uint8> solidMask(sizeZ, 0);
                int const firstSolid = first ? int(first->smin) : sizeZ;
                for (int z = 0; z < std::min(firstSolid, sizeZ); ++z)
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
        std::vector<float> RawV9;
        std::vector<float> RawV8;
        std::array<uint16, 256> Holes{};
        bool HasHoles = false;
        uint16 LiquidType = 0;
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
                    mapData.RawV9.resize(v9.size());
                    mapData.RawV8.resize(v8.size());
                    for (size_t i = 0; i < v9.size(); ++i)
                    {
                        mapData.RawV9[i] = float(v9[i]);
                        mapData.V9[i] = float(v9[i]) * mapData.HeightMultiplier + gridHeight;
                    }
                    for (size_t i = 0; i < v8.size(); ++i)
                    {
                        mapData.RawV8[i] = float(v8[i]);
                        mapData.V8[i] = float(v8[i]) * mapData.HeightMultiplier + gridHeight;
                    }
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
                    mapData.RawV9.resize(v9.size());
                    mapData.RawV8.resize(v8.size());
                    for (size_t i = 0; i < v9.size(); ++i)
                    {
                        mapData.RawV9[i] = float(v9[i]);
                        mapData.V9[i] = float(v9[i]) * mapData.HeightMultiplier + gridHeight;
                    }
                    for (size_t i = 0; i < v8.size(); ++i)
                    {
                        mapData.RawV8[i] = float(v8[i]);
                        mapData.V8[i] = float(v8[i]) * mapData.HeightMultiplier + gridHeight;
                    }
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
                    mapData.RawV9 = mapData.V9;
                    mapData.RawV8 = mapData.V8;
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
            mapData.LiquidType = liquidType;
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

        float const h1 = mapData.RawV9[xInt * 129 + yInt];
        float const h2 = mapData.RawV9[(xInt + 1) * 129 + yInt];
        float const h3 = mapData.RawV9[xInt * 129 + yInt + 1];
        float const h4 = mapData.RawV9[(xInt + 1) * 129 + yInt + 1];
        float const h5 = 2.0f * mapData.RawV8[xInt * 128 + yInt];

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

        return (a * x + b * y + c) * mapData.HeightMultiplier + mapData.GridHeight;
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
        float const tileOriginX = (32.0f - float(tileX)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        float const tileOriginY = (32.0f - float(tileY)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        float const tileMaxX = tileOriginX + MMAP::GRID_SIZE;
        float const tileMaxY = tileOriginY + MMAP::GRID_SIZE;
        float originX = tileOriginX;
        float originY = tileOriginY;
        int width = int(std::ceil(MMAP::GRID_SIZE / cellSize));
        int height = int(std::ceil(MMAP::GRID_SIZE / cellSize));

        if (options.HasBox)
        {
            float const boxMinX = std::clamp(options.BoxMinX, tileOriginX, tileMaxX);
            float const boxMaxX = std::clamp(options.BoxMaxX, tileOriginX, tileMaxX);
            float const boxMinY = std::clamp(options.BoxMinY, tileOriginY, tileMaxY);
            float const boxMaxY = std::clamp(options.BoxMaxY, tileOriginY, tileMaxY);
            if (boxMaxX <= boxMinX || boxMaxY <= boxMinY)
            {
                std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                          << " (empty box)\n";
                return true;
            }
            originX = tileOriginX + std::floor((boxMinX - tileOriginX) / cellSize) * cellSize;
            originY = tileOriginY + std::floor((boxMinY - tileOriginY) / cellSize) * cellSize;
            width = std::max(1, int(std::ceil((boxMaxX - originX) / cellSize)));
            height = std::max(1, int(std::ceil((boxMaxY - originY) / cellSize)));
        }

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

        if (!options.HasWaterLevel && mapData.LiquidType != 0 &&
            !IsWaterLiquid(mapData.LiquidType))
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (non-water liquid type " << mapData.LiquidType << ")\n";
            return true;
        }

        float minZ = options.HasMinZ ? options.MinZ : minTerrain - cellSize;
        if (!options.HasMinZ && options.HasMaxDepth)
            minZ = std::max(minZ, maxLiquid - options.MaxDepth);
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

    bool GenerateTerrainAirTile(Options const& options, uint32 tileX, uint32 tileY)
    {
        TerrainMapFile mapData;
        if (!LoadTerrainMap(options, tileX, tileY, mapData) || !mapData.HasHeight)
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (no readable terrain)\\n";
            return true;
        }

        float minTerrain = std::numeric_limits<float>::max();
        float maxTerrain = -std::numeric_limits<float>::max();
        for (float heightValue : mapData.V9)
        {
            minTerrain = std::min(minTerrain, heightValue);
            maxTerrain = std::max(maxTerrain, heightValue);
        }
        if (minTerrain > maxTerrain)
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (empty terrain)\\n";
            return true;
        }

        float const cellSize = options.CellSize;
        float const originX = (32.0f - float(tileX)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        float const originY = (32.0f - float(tileY)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        int const width = int(std::ceil(MMAP::GRID_SIZE / cellSize));
        int const height = int(std::ceil(MMAP::GRID_SIZE / cellSize));
        float const minZ = std::max(-1000.0f, minTerrain - 10.0f);
        // A single air interval is stored per column, so a high ceiling does not grow the
        // payload. Cover the full 3.3.5 flight envelope, not just this tile's peaks.
        float const maxZ = 4000.0f;
        int const sizeZ = std::max(1, int(std::ceil((maxZ - minZ) / cellSize)));
        size_t const cellCount = size_t(width) * size_t(height) * size_t(sizeZ);
        if (sizeZ > 32767 || cellCount > kMaxVoxelCells)
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (air grid too large: " << cellCount << ")\\n";
            return true;
        }

        auto const cellIndex = [width, height, sizeZ](int x, int y, int z) -> size_t
        {
            return (size_t(z) * size_t(height) + size_t(y)) * size_t(width) + size_t(x);
        };

        Nav3DTile tile;
        Nav3DTileHeader& header = tile.Header();
        header.MapId = options.MapId;
        header.TileX = uint16(tileX);
        header.TileY = uint16(tileY);
        header.Medium = Nav3D::MediumAir;
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
        size_t airColumns = 0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                float floor = -1000.0f;
                // Sample the cell footprint so a 4-yd air cell does not cut a terrain spike.
                for (int sy = 0; sy < 3; ++sy)
                {
                    for (int sx = 0; sx < 3; ++sx)
                    {
                        float const wx = originX + (float(x) + (float(sx) + 0.5f) / 3.0f) * cellSize;
                        float const wy = originY + (float(y) + (float(sy) + 0.5f) / 3.0f) * cellSize;
                        float const h = TerrainHeight(mapData, wx, wy);
                        if (h > -1000.0f)
                            floor = std::max(floor, h);
                    }
                }
                if (floor <= -1000.0f)
                    continue;

                int const firstZ = std::clamp(int(std::ceil((floor + kBodyHalfHeight - minZ) / cellSize)), 0,
                                              sizeZ);
                if (firstZ >= sizeZ)
                    continue;

                Nav3DTileColumn& column = tile.Columns()[size_t(y) * size_t(width) + size_t(x)];
                Nav3DTileInterval interval;
                interval.ZMin = int16(firstZ);
                interval.ZMax = int16(sizeZ - 1);
                interval.Clearance = 16;
                interval.Flags = 0;
                column.IntervalOffset = uint32(tile.Intervals().size());
                column.IntervalCount = 1;
                tile.Intervals().push_back(interval);
                ++airColumns;
            }
        }

        std::string const relative = Acore::StringFormat("map{:04}/{}_{}_air.n3t", options.MapId, tileX,
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

        std::cout << "wrote " << outputFile << " (" << airColumns << " air columns, " << sizeZ
                  << " z cells, " << tile.Intervals().size() << " intervals)\n";
        return true;
    }

    bool GenerateAirVolumeTile(Options const& options, uint32 tileX, uint32 tileY)
    {
        TerrainMapFile mapData;
        if (!LoadTerrainMap(options, tileX, tileY, mapData))
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (no terrain map for air volume)\n";
            return true;
        }

        MMAP::TerrainBuilder terrainBuilder(options.DataDir, false);
        MMAP::MeshData meshData;
        for (int offsetX = -1; offsetX <= 1; ++offsetX)
        {
            for (int offsetY = -1; offsetY <= 1; ++offsetY)
            {
                int const wmoX = int(tileX) + offsetX;
                int const wmoY = int(tileY) + offsetY;
                if (wmoX < 0 || wmoY < 0 || wmoX >= 64 || wmoY >= 64)
                    continue;
                terrainBuilder.loadVMap(options.MapId, uint32(wmoX), uint32(wmoY), meshData);
            }
        }
        MMAP::TerrainBuilder::cleanVertices(meshData.solidVerts, meshData.solidTris);

        // TerrainBuilder writes VMAP geometry in the mmaps frame:
        //     mmaps = iPos - 32*GRID_SIZE
        // The server/world frame is the mirror:
        //     server = 32*GRID_SIZE - iPos
        // Convert before rasterizing so WMO tunnel/cave spans land on the same
        // coordinates as the .map terrain and the runtime .n3t tiles.
        for (int i = 0; i + 2 < meshData.solidVerts.size(); i += 3)
        {
            meshData.solidVerts[i] = -meshData.solidVerts[i];
            meshData.solidVerts[i + 1] = -meshData.solidVerts[i + 1];
        }
        for (int i = 0; i + 2 < meshData.liquidVerts.size(); i += 3)
        {
            meshData.liquidVerts[i] = -meshData.liquidVerts[i];
            meshData.liquidVerts[i + 1] = -meshData.liquidVerts[i + 1];
        }

        float const cellSize = options.CellSize;
        // Align the volumetric air tiles to a world-wide lattice. A composite multi-tile
        // search addresses nodes by world position; if each tile kept its own half-cell
        // offset, a global node would not land on the same cell in neighbouring tiles.
        float const rawOriginX = (32.0f - float(tileX)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        float const rawOriginY = (32.0f - float(tileY)) * MMAP::GRID_SIZE - MMAP::GRID_SIZE;
        float const originX = std::floor(rawOriginX / cellSize) * cellSize;
        float const originY = std::floor(rawOriginY / cellSize) * cellSize;
        int const width =
            std::max(1, int(std::ceil((rawOriginX + MMAP::GRID_SIZE - originX) / cellSize)));
        int const height =
            std::max(1, int(std::ceil((rawOriginY + MMAP::GRID_SIZE - originY) / cellSize)));

        float minTerrain = std::numeric_limits<float>::max();
        for (float heightValue : mapData.V9)
            minTerrain = std::min(minTerrain, heightValue);
        float const minZ =
            std::floor(std::max(-1000.0f, minTerrain - 10.0f) / cellSize) * cellSize;
        float const maxZ = 4000.0f;
        int const sizeZ = std::max(1, int(std::ceil((maxZ - minZ) / cellSize)));
        size_t const cellCount = size_t(width) * size_t(height) * size_t(sizeZ);
        if (sizeZ > 32767 || cellCount > kMaxVoxelCells)
        {
            std::cout << "skip map " << options.MapId << " tile " << tileX << ":" << tileY
                      << " (air volume too large: " << cellCount << ")\n";
            return true;
        }

        rcContext context;
        rcHeightfield* solid = rcAllocHeightfield();
        if (!solid)
            return false;

        float bmin[3] = {originX, minZ, originY};
        float bmax[3] = {originX + float(width) * cellSize, maxZ,
                         originY + float(height) * cellSize};
        if (!rcCreateHeightfield(&context, *solid, width, height, bmin, bmax, cellSize, cellSize))
        {
            rcFreeHeightField(solid);
            return false;
        }

        if (meshData.solidVerts.size() && meshData.solidTris.size())
        {
            std::vector<unsigned char> areas(meshData.solidTris.size() / 3, RC_WALKABLE_AREA);
            if (!rcRasterizeTriangles(&context, meshData.solidVerts.getCArray(),
                                      meshData.solidVerts.size() / 3, meshData.solidTris.getCArray(),
                                      areas.data(), areas.size(), *solid))
            {
                rcFreeHeightField(solid);
                return false;
            }
        }

        auto const cellIndex = [width, height, sizeZ](int x, int y, int z) -> size_t
        {
            return (size_t(z) * size_t(height) + size_t(y)) * size_t(width) + size_t(x);
        };

        Nav3DTile tile;
        Nav3DTileHeader& header = tile.Header();
        header.MapId = options.MapId;
        header.TileX = uint16(tileX);
        header.TileY = uint16(tileY);
        header.Medium = Nav3D::MediumAir;
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
        header.GeneratorVersion = 6;
        header.GeometryHash = 0;

        tile.Columns().resize(size_t(width) * size_t(height));
        size_t airIntervals = 0;
        size_t airColumns = 0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                float const wx = originX + (float(x) + 0.5f) * cellSize;
                float const wy = originY + (float(y) + 0.5f) * cellSize;
                float const terrain = TerrainHeight(mapData, wx, wy);

                int const floorCell =
                    terrain > -1000.0f
                        ? std::clamp(int(std::floor((terrain - minZ) / cellSize)), 0, sizeZ - 1)
                        : -1;

                struct SolidSpan
                {
                    int Lo;
                    int Hi;
                    bool Wmo;
                };
                std::vector<SolidSpan> spans;
                for (rcSpan const* span = solid->spans[y * width + x]; span; span = span->next)
                {
                    int const lo = std::max(0, int(span->smin));
                    int const hi = std::min(sizeZ, int(span->smax));
                    if (lo < hi)
                        spans.push_back({lo, hi, true});
                }
                if (floorCell >= 0)
                    spans.push_back({floorCell, floorCell + 1, false});

                std::sort(spans.begin(), spans.end(),
                          [](SolidSpan const& left, SolidSpan const& right)
                          { return left.Lo < right.Lo; });
                std::vector<SolidSpan> merged;
                for (SolidSpan const& span : spans)
                {
                    if (merged.empty() || span.Lo > merged.back().Hi)
                        merged.push_back(span);
                    else if (span.Hi > merged.back().Hi)
                    {
                        merged.back().Hi = span.Hi;
                        merged.back().Wmo = span.Wmo;
                    }
                }

                Nav3DTileColumn& column = tile.Columns()[size_t(y) * size_t(width) + size_t(x)];
                int cursor = 0;
                bool first = true;
                bool any = false;
                int const bodyPad = int(std::ceil(kBodyHalfHeight / cellSize));
                auto const emitAir = [&](int lo, int hi)
                {
                    if (lo > hi)
                        return;
                    Nav3DTileInterval interval;
                    interval.ZMin = int16(lo);
                    interval.ZMax = int16(hi);
                    interval.Clearance = 16;
                    interval.Flags = 0;
                    if (column.IntervalCount == 0)
                        column.IntervalOffset = uint32(tile.Intervals().size());
                    column.IntervalCount = uint16(column.IntervalCount + 1);
                    tile.Intervals().push_back(interval);
                    ++airIntervals;
                    any = true;
                };
                for (SolidSpan const& solidSpan : merged)
                {
                    // Only gaps bounded above by a WMO ceiling are cavities. The gap
                    // between a WMO ceiling and the terrain lid is rock, not air.
                    if (!first && solidSpan.Lo > cursor && solidSpan.Wmo)
                        emitAir(cursor + bodyPad, solidSpan.Lo - 1);
                    cursor = std::max(cursor, solidSpan.Hi);
                    first = false;
                }
                if (cursor < sizeZ)
                    emitAir(cursor + bodyPad, sizeZ - 1);
                if (any)
                    ++airColumns;
            }
        }

        std::string const relative = Acore::StringFormat("map{:04}/{}_{}_air.n3t", options.MapId, tileX,
                                                         tileY);
        std::filesystem::path const outputFile = std::filesystem::path(options.OutputDir) / relative;
        std::error_code ec;
        std::filesystem::create_directories(outputFile.parent_path(), ec);
        if (ec)
        {
            std::cerr << "cannot create " << outputFile.parent_path() << ": " << ec.message() << "\n";
            rcFreeHeightField(solid);
            return false;
        }
        bool const saved = tile.Save(outputFile.string());
        rcFreeHeightField(solid);
        if (!saved)
        {
            std::cerr << "failed to write " << outputFile << "\n";
            return false;
        }

        std::cout << "wrote " << outputFile << " (" << airColumns << " air columns, "
                  << airIntervals << " intervals, " << sizeZ << " z cells)\n";
        return true;
    }

    bool GenerateAirTreeDetailTile(Options const& options)
    {
        if (!options.HasBox || options.Tiles.empty() || options.Seeds.empty())
        {
            std::cerr << "air-tree-detail requires --box, --tile and --seed\n";
            return false;
        }

        float const cellSize = options.CellSize > 0.0f ? options.CellSize : 1.0f;
        auto const headerTile = *options.Tiles.begin();
        uint32 const tileX = headerTile.first;
        uint32 const tileY = headerTile.second;

        float const originX = std::floor(options.BoxMinX / cellSize) * cellSize;
        float const originY = std::floor(options.BoxMinY / cellSize) * cellSize;
        float const originZ = std::floor(options.BoxMinZ / cellSize) * cellSize;
        int const width = std::max(1, int(std::ceil((options.BoxMaxX - originX) / cellSize)));
        int const height = std::max(1, int(std::ceil((options.BoxMaxY - originY) / cellSize)));
        int const sizeZ = std::max(1, int(std::ceil((options.BoxMaxZ - originZ) / cellSize)));
        uint64 const cellCount = uint64(width) * uint64(height) * uint64(sizeZ);
        if (cellCount > 256ull * 1024ull * 1024ull || sizeZ > 32767)
        {
            std::cerr << "air-tree-detail box too large: " << cellCount << " cells\n";
            return false;
        }

        std::string basePath = options.DataDir;
        if (!basePath.empty() && basePath.back() != '/')
            basePath += '/';
        basePath += "vmaps/";

        VMAP::StaticMapTree tree(options.MapId, basePath);
        if (!tree.InitMap(VMAP::VMapMgr2::getMapFileName(options.MapId)))
        {
            std::cerr << "cannot init StaticMapTree\n";
            return false;
        }
        for (int ty = int(tileY) - 1; ty <= int(tileY) + 1; ++ty)
            if (ty >= 0 && ty < 64)
                tree.LoadMapTile(tileX, uint32(ty));

        TerrainMapFile terrainLow;
        TerrainMapFile terrainHigh;
        bool const hasLow = tileY > 0 && LoadTerrainMap(options, tileX, tileY - 1, terrainLow);
        bool const hasHigh = LoadTerrainMap(options, tileX, tileY, terrainHigh);
        float const splitY = (32.0f - float(tileY)) * 533.3333f;

        auto const terrainAt = [&](float x, float y) -> float
        {
            // tileY-1 covers [splitY, splitY + GRID); tileY covers [splitY - GRID, splitY).
            if (y >= splitY)
                return hasLow ? TerrainHeight(terrainLow, x, y) : -1000.0f;
            return hasHigh ? TerrainHeight(terrainHigh, x, y) : -1000.0f;
        };

        // MMap is a diagnostic cross-check only; the solid-volume gates above use
        // StaticMapTree directly, so a missing mmaps set must not fail the bake.
        std::vector<float> mmapFloor(size_t(width) * size_t(height), -1e9f);
        struct NavMeshDeleter { void operator()(dtNavMesh* mesh) const { dtFreeNavMesh(mesh); } };
        struct QueryDeleter { void operator()(dtNavMeshQuery* query) const { dtFreeNavMeshQuery(query); } };
        std::shared_ptr<dtNavMesh> navMesh;
        std::shared_ptr<dtNavMeshQuery> navQuery;
        if (options.AirDetailDiag)
        {
            std::string const mmapFile = Acore::StringFormat("{}/mmaps/{:03}.mmap", options.DataDir,
                                                              options.MapId);
            FILE* file = fopen(mmapFile.c_str(), "rb");
            if (file)
            {
                dtNavMeshParams params;
                if (fread(&params, sizeof(dtNavMeshParams), 1, file) == 1)
                {
                    dtNavMesh* mesh = dtAllocNavMesh();
                    if (mesh && dtStatusSucceed(mesh->init(&params)))
                        navMesh = std::shared_ptr<dtNavMesh>(mesh, NavMeshDeleter());
                    else if (mesh)
                        dtFreeNavMesh(mesh);
                }
                fclose(file);
            }
            if (navMesh)
            {
                for (int ty = int(tileY) - 1; ty <= int(tileY) + 1; ++ty)
                {
                    if (ty < 0 || ty >= 64)
                        continue;
                    std::string const tileFile = Acore::StringFormat("{}/mmaps/{:03}{:02}{:02}.mmtile",
                                                                      options.DataDir, options.MapId,
                                                                      tileX, ty);
                    FILE* tile = fopen(tileFile.c_str(), "rb");
                    if (!tile)
                        continue;
                    MmapTileHeader header;
                    if (fread(&header, sizeof(MmapTileHeader), 1, tile) == 1 &&
                        header.mmapMagic == MMAP_MAGIC && header.mmapVersion == MMAP_VERSION)
                    {
                        unsigned char* data = (unsigned char*)dtAlloc(header.size, DT_ALLOC_PERM);
                        if (data && fread(data, header.size, 1, tile) == 1)
                        {
                            dtTileRef ref = 0;
                            if (dtStatusFailed(navMesh->addTile(data, header.size, DT_TILE_FREE_DATA, 0,
                                                               &ref)))
                                dtFree(data);
                        }
                        else if (data)
                            dtFree(data);
                    }
                    fclose(tile);
                }
                dtNavMeshQuery* query = dtAllocNavMeshQuery();
                if (query && dtStatusSucceed(query->init(navMesh.get(), 1024)))
                    navQuery = std::shared_ptr<dtNavMeshQuery>(query, QueryDeleter());
                else if (query)
                    dtFreeNavMeshQuery(query);
            }
        }
        if (navQuery)
        {
            dtQueryFilter navFilter;
            navFilter.setIncludeFlags(0xFFFF);
            for (int y = 0; y < height; ++y)
            {
            for (int x = 0; x < width; ++x)
            {
                float const wx = originX + (float(x) + 0.5f) * cellSize;
                float const wy = originY + (float(y) + 0.5f) * cellSize;
                float const pt[3] = {wy, originZ + 1.0f, wx};
                float const extents[3] = {2.0f, 2000.0f, 2.0f};
                dtPolyRef ref = 0;
                float closest[3];
                    if (dtStatusSucceed(navQuery->findNearestPoly(pt, extents, &navFilter, &ref, closest)) &&
                        ref)
                    {
                        float height = 0.0f;
                        if (dtStatusSucceed(navQuery->getPolyHeight(ref, closest, &height)))
                            mmapFloor[size_t(y) * size_t(width) + size_t(x)] = height;
                    }
                }
            }
        }

        auto const mirror = [](float x, float y, float z) -> G3D::Vector3
        {
            return G3D::Vector3(17066.0f - x, 17066.0f - y, z);
        };

        constexpr float bodyRadius = 0.45f;
        constexpr float bodyDiagonal = 0.6364f;
        constexpr float bodyHalfHeight = 0.8f;
        struct Ray
        {
            float X;
            float Y;
            float Z;
            float Distance;
        };
        static constexpr Ray rays[10] = {
            {1.0f, 0.0f, 0.0f, bodyRadius},         {-1.0f, 0.0f, 0.0f, bodyRadius},
            {0.0f, 1.0f, 0.0f, bodyRadius},         {0.0f, -1.0f, 0.0f, bodyRadius},
            {0.7071f, 0.7071f, 0.0f, bodyDiagonal}, {-0.7071f, 0.7071f, 0.0f, bodyDiagonal},
            {0.7071f, -0.7071f, 0.0f, bodyDiagonal}, {-0.7071f, -0.7071f, 0.0f, bodyDiagonal},
            {0.0f, 0.0f, 1.0f, bodyHalfHeight},     {0.0f, 0.0f, -1.0f, bodyHalfHeight}};

        auto const airAt = [&](float x, float y, float z) -> bool
        {
            G3D::Vector3 const point = mirror(x, y, z);
            VMAP::LocationInfo info;
            bool const hasWmo = tree.GetLocationInfo(point, info);
            float const wmoFloor = tree.getHeight(point, 5000.0f);
            float const terrain = terrainAt(x, y);
            bool const belowTerrain = terrain > -1000.0f && z < terrain - 1.0f;
            int const ix = std::clamp(int((x - originX) / cellSize), 0, width - 1);
            int const iy = std::clamp(int((y - originY) / cellSize), 0, height - 1);
            float const floor = mmapFloor[size_t(iy) * size_t(width) + size_t(ix)];
            float const columnTop = belowTerrain && std::isfinite(terrain)
                                        ? tree.getHeight(mirror(x, y, terrain), 5000.0f)
                                        : -std::numeric_limits<float>::infinity();
            char const* reject = nullptr;
            char const* downState = "n/a";
            char const* upState = "n/a";
            char const* bodyState = "n/a";
            if (belowTerrain)
            {
                // The cell must sit above a real WMO floor. This is the solid-volume
                // floor test; MMap is kept only as a diagnostic cross-check, because a
                // multi-level WMO room can have a nearby floor far above the navmesh
                // surface and still be valid air.
                if (!hasWmo || !std::isfinite(wmoFloor) || wmoFloor > z + 0.5f)
                    reject = "wmo-floor";
                else
                {
                    // The shaft down to the WMO floor must be clear. Above a WMO/cave
                    // ceiling the downward ray hits the roof, so rock between the
                    // ceiling and the outdoor terrain is rejected.
                    G3D::Vector3 const down = mirror(x, y, wmoFloor + 0.5f);
                    bool const downClear =
                        tree.isInLineOfSight(point, down, VMAP::ModelIgnoreFlags::Nothing);
                    downState = downClear ? "clear" : "blocked";
                    if (!downClear)
                        reject = "down-los";
                    else
                    {
                        // The cell must also be under a WMO/tunnel ceiling. Above the
                        // tunnel roof the upward ray reaches the terrain with no WMO hit;
                        // that is solid cliff/rock, not cave air.
                        G3D::Vector3 const up = mirror(x, y, terrain + 1.0f);
                        bool const upClear =
                            std::isfinite(terrain) &&
                            tree.isInLineOfSight(point, up, VMAP::ModelIgnoreFlags::Nothing);
                        upState = upClear ? "clear" : "blocked";
                        if (upClear)
                            reject = "up-los";
                        // Highest WMO surface below the outdoor terrain: above the tunnel
                        // roof the column-top test rejects rock between roof and terrain.
                        else if (std::isfinite(columnTop) && z > columnTop + 0.5f)
                            reject = "wmo-column-top";
                    }
                }
            }
            if (!reject)
            {
                for (Ray const& ray : rays)
                {
                    G3D::Vector3 const end = point + G3D::Vector3(ray.X * ray.Distance,
                                                                 ray.Y * ray.Distance,
                                                                 ray.Z * ray.Distance);
                    if (!tree.isInLineOfSight(point, end, VMAP::ModelIgnoreFlags::Nothing))
                    {
                        bodyState = "blocked";
                        reject = "body";
                        break;
                    }
                }
                if (!reject)
                    bodyState = "clear";
            }
            if (options.AirDetailDiag)
            {
                std::cout << "diag airAt (" << x << "," << y << "," << z << ")"
                          << " terrain=" << terrain << " below=" << belowTerrain
                          << " hasWmo=" << hasWmo << " wmoFloor=" << wmoFloor
                          << " mmapFloor=" << floor << " down=" << downState
                          << " up=" << upState << " columnTop=" << columnTop
                          << " body=" << bodyState << " -> " << (reject ? reject : "air") << "\n";
            }
            return !reject;
        };

        auto const boxIndex = [width, height](int x, int y, int z) -> size_t
        {
            return (size_t(z) * size_t(height) + size_t(y)) * size_t(width) + size_t(x);
        };
        if (options.AirDetailDiag)
        {
            for (auto const& seed : options.Seeds)
                airAt(seed[0], seed[1], seed[2]);
            return true;
        }

        std::vector<uint8_t> air(cellCount, 0);
        std::vector<int8_t> cache(cellCount, 0);

        uint32 const jobs = std::clamp<uint32>(options.Jobs, 1, 32);
        std::atomic<size_t> nextCell{0};
        auto const classify = [&]()
        {
            while (true)
            {
                size_t const index = nextCell.fetch_add(1);
                if (index >= cellCount)
                    break;
                int const x = int(index % size_t(width));
                int const y = int((index / size_t(width)) % size_t(height));
                int const z = int(index / (size_t(width) * size_t(height)));
                bool const value = airAt(originX + (float(x) + 0.5f) * cellSize,
                                         originY + (float(y) + 0.5f) * cellSize,
                                         originZ + (float(z) + 0.5f) * cellSize);
                cache[index] = value ? 1 : -1;
            }
        };
        if (jobs <= 1)
            classify();
        else
        {
            std::vector<std::thread> workers;
            workers.reserve(jobs);
            for (uint32 i = 0; i < jobs; ++i)
                workers.emplace_back(classify);
            for (std::thread& worker : workers)
                worker.join();
        }

        auto const cachedAir = [&](int x, int y, int z) -> bool
        {
            return cache[boxIndex(x, y, z)] > 0;
        };

        std::deque<std::array<int, 3>> queue;
        size_t liveCells = 0;
        for (auto const& seed : options.Seeds)
        {
            int const sx = std::clamp(int(std::floor((seed[0] - originX) / cellSize)), 0, width - 1);
            int const sy = std::clamp(int(std::floor((seed[1] - originY) / cellSize)), 0, height - 1);
            int const sz = std::clamp(int(std::floor((seed[2] - originZ) / cellSize)), 0, sizeZ - 1);
            bool found = false;
            for (int radius = 0; radius <= 2 && !found; ++radius)
            {
                for (int dx = -radius; dx <= radius && !found; ++dx)
                {
                    for (int dy = -radius; dy <= radius && !found; ++dy)
                    {
                        for (int dz = -radius; dz <= radius && !found; ++dz)
                        {
                            int const x = sx + dx;
                            int const y = sy + dy;
                            int const z = sz + dz;
                            if (x < 0 || x >= width || y < 0 || y >= height || z < 0 || z >= sizeZ)
                                continue;
                            if (!cachedAir(x, y, z))
                                continue;
                            size_t const index = boxIndex(x, y, z);
                            if (air[index])
                                continue;
                            air[index] = 1;
                            ++liveCells;
                            queue.push_back({x, y, z});
                            found = true;
                        }
                    }
                }
            }
            if (!found)
                std::cerr << "seed has no air: " << seed[0] << "," << seed[1] << "," << seed[2] << "\n";
        }
        if (queue.empty())
        {
            std::cerr << "no air seeds found\n";
            return false;
        }

        static constexpr int neighbors[6][3] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        while (!queue.empty())
        {
            auto const current = queue.front();
            queue.pop_front();
            G3D::Vector3 const from = mirror(originX + (float(current[0]) + 0.5f) * cellSize,
                                             originY + (float(current[1]) + 0.5f) * cellSize,
                                             originZ + (float(current[2]) + 0.5f) * cellSize);
            for (auto const& offset : neighbors)
            {
                int const x = current[0] + offset[0];
                int const y = current[1] + offset[1];
                int const z = current[2] + offset[2];
                if (x < 0 || x >= width || y < 0 || y >= height || z < 0 || z >= sizeZ)
                    continue;
                size_t const index = boxIndex(x, y, z);
                if (air[index] || !cachedAir(x, y, z))
                    continue;
                G3D::Vector3 const to = mirror(originX + (float(x) + 0.5f) * cellSize,
                                               originY + (float(y) + 0.5f) * cellSize,
                                               originZ + (float(z) + 0.5f) * cellSize);
                if (!tree.isInLineOfSight(from, to, VMAP::ModelIgnoreFlags::Nothing))
                    continue;
                air[index] = 1;
                ++liveCells;
                queue.push_back({x, y, z});
            }
        }

        Nav3DTile tile;
        Nav3DTileHeader& header = tile.Header();
        header.MapId = options.MapId;
        header.TileX = uint16(tileX);
        header.TileY = uint16(tileY);
        header.Medium = Nav3D::MediumAirDetail;
        header.CellSize = cellSize;
        header.OriginX = originX;
        header.OriginY = originY;
        header.OriginZ = originZ;
        header.ExtentX = float(width) * cellSize;
        header.ExtentY = float(height) * cellSize;
        header.ExtentZ = float(sizeZ) * cellSize;
        header.SizeX = uint32(width);
        header.SizeY = uint32(height);
        header.SizeZ = uint32(sizeZ);
        // The detail tile is authoritative for its box: the bake already encodes the
        // static solid volume, and a live-map re-check would make the result depend on
        // which vmap grids happen to be loaded around the player. Clearing FlagLiveLos
        // keeps runtime A* on the baked volume only.
        header.Flags |= Nav3D::FlagHasSolid;
        header.GeneratorVersion = 6;
        header.GeometryHash = 0;

        tile.Columns().resize(size_t(width) * size_t(height));
        size_t liveColumns = 0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                Nav3DTileColumn& column = tile.Columns()[size_t(y) * size_t(width) + size_t(x)];
                bool any = false;
                for (int z = 0; z < sizeZ;)
                {
                    if (!air[boxIndex(x, y, z)])
                    {
                        ++z;
                        continue;
                    }
                    int const start = z;
                    while (z < sizeZ && air[boxIndex(x, y, z)])
                        ++z;
                    column.IntervalOffset = column.IntervalCount == 0
                                                ? uint32(tile.Intervals().size())
                                                : column.IntervalOffset;
                    Nav3DTileInterval interval;
                    interval.ZMin = int16(start);
                    interval.ZMax = int16(z - 1);
                    interval.Clearance = 16;
                    interval.Flags = 0;
                    tile.Intervals().push_back(interval);
                    column.IntervalCount = uint16(column.IntervalCount + 1);
                    any = true;
                }
                if (any)
                    ++liveColumns;
            }
        }

        std::string const relative = Acore::StringFormat("map{:04}/{:02}_{:02}_airdetail.n3t",
                                                         options.MapId, tileX, tileY);
        std::filesystem::path const outputFile =
            std::filesystem::path(options.OutputDir) / relative;
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

        std::cout << "wrote " << outputFile << " (airdetail " << liveColumns << " columns, "
                  << liveCells << " cells, " << tile.Intervals().size() << " intervals, "
                  << width << "x" << height << "x" << sizeZ << ")\n";
        return true;
    }

    bool ProbeMap(Options const& options)
    {
        std::string basePath = options.DataDir;
        if (!basePath.empty() && basePath.back() != '/')
            basePath += '/';
        basePath += "vmaps/";
        std::string const mapFileName = VMAP::VMapMgr2::getMapFileName(options.MapId);

        std::cout << "probe map " << options.MapId << " point=(" << options.ProbeX << ","
                  << options.ProbeY << "," << options.ProbeZ << ") base=" << basePath << "\n";

        int const probeTileX = std::clamp(int(std::floor(32.0f - options.ProbeX / 533.3333f)), 0, 63);
        int const probeTileY = std::clamp(int(std::floor(32.0f - options.ProbeY / 533.3333f)), 0, 63);
        std::array<std::pair<uint32, uint32>, 5> const orders = {
            {{18, 32}, {32, 18}, {18, 33}, {33, 18}, {uint32(probeTileX), uint32(probeTileY)}}};
        for (auto const& order : orders)
        {
            VMAP::StaticMapTree tree(options.MapId, basePath);
            if (!tree.InitMap(mapFileName))
            {
                std::cout << "tile " << order.first << ":" << order.second << " InitMap failed\n";
                continue;
            }
            tree.LoadMapTile(order.first, order.second);
            VMAP::ModelInstance* models = nullptr;
            uint32_t count = 0;
            tree.GetModelInstances(models, count);
            std::cout << "tile " << order.first << ":" << order.second << " models=" << count << "\n";
            for (uint32_t i = 0; i < count; ++i)
            {
                std::string lower = models[i].name;
                std::transform(lower.begin(), lower.end(), lower.begin(),
                               [](unsigned char c) { return char(std::tolower(c)); });
                if (lower.find("titan") == std::string::npos && lower.find("machine") == std::string::npos)
                    continue;
                std::cout << "  " << models[i].name << " pos=(" << models[i].iPos.x << ","
                          << models[i].iPos.y << "," << models[i].iPos.z << ") scale="
                          << models[i].iScale << " bound=(" << models[i].iBound.low().x << ","
                          << models[i].iBound.low().y << "," << models[i].iBound.low().z << ")-("
                          << models[i].iBound.high().x << "," << models[i].iBound.high().y << ","
                          << models[i].iBound.high().z << ")\n";
            }

            // Probe WMO liquid groups (WmoLiquid) for WMO models in the tile.
            for (uint32_t i = 0; i < count; ++i)
            {
                std::string lower = models[i].name;
                std::transform(lower.begin(), lower.end(), lower.begin(),
                               [](unsigned char c) { return char(std::tolower(c)); });
                if (lower.find(".wmo") == std::string::npos)
                    continue;
                VMAP::WorldModel* worldModel = models[i].getWorldModel();
                if (!worldModel)
                    continue;
                std::vector<VMAP::GroupModel> groups;
                worldModel->GetGroupModels(groups);
                std::cout << "  liquid model " << models[i].name << " groups=" << groups.size() << "\n";
                for (size_t gi = 0; gi < groups.size(); ++gi)
                {
                    std::vector<G3D::Vector3> verts;
                    std::vector<VMAP::MeshTriangle> tris;
                    VMAP::WmoLiquid* liquid = nullptr;
                    groups[gi].GetMeshData(verts, tris, liquid);
                    if (!liquid)
                        continue;
                    uint32 tilesX = 0;
                    uint32 tilesY = 0;
                    G3D::Vector3 corner;
                    liquid->GetPosInfo(tilesX, tilesY, corner);
                    std::cout << "    group " << gi << " liquid type=" << liquid->GetType()
                              << " flags=" << (liquid->GetFlagsStorage() ? "yes" : "null")
                              << " tiles=" << tilesX << "x" << tilesY << " corner=(" << corner.x << ","
                              << corner.y << "," << corner.z << ")\n";
                }
            }

            struct Candidate
            {
                char const* Name;
                G3D::Vector3 Point;
            };
            std::array<Candidate, 5> const points = {{
                {"server", G3D::Vector3(options.ProbeX, options.ProbeY, options.ProbeZ)},
                {"minus-17066", G3D::Vector3(options.ProbeX - 17066.0f,
                                            options.ProbeY - 17066.0f, options.ProbeZ)},
                {"mirror-17066", G3D::Vector3(17066.0f - options.ProbeX,
                                             17066.0f - options.ProbeY, options.ProbeZ)},
                {"swap", G3D::Vector3(options.ProbeY, options.ProbeX, options.ProbeZ)},
                {"swap-mirror", G3D::Vector3(17066.0f - options.ProbeY,
                                             17066.0f - options.ProbeX, options.ProbeZ)},
            }};
            for (Candidate const& candidate : points)
            {
                float const height = tree.getHeight(candidate.Point, 5000.0f);
                VMAP::LocationInfo info;
                bool const found = tree.GetLocationInfo(candidate.Point, info);
                std::cout << "  height " << candidate.Name << "=(" << candidate.Point.x << ","
                          << candidate.Point.y << "," << candidate.Point.z << ") -> " << height
                          << " loc=" << (found ? 1 : 0) << " ground=" << info.ground_Z
                          << " root=" << info.rootId << "\n";
            }
        }
        return true;
    }

    // Keep this in sync with the output paths in the generators above.
    std::string RelativeOutputPath(Options const& options, uint32 tileX, uint32 tileY)
    {
        if (options.AirTreeDetail)
            return Acore::StringFormat("map{:04}/{:02}_{:02}_airdetail.n3t", options.MapId, tileX,
                                       tileY);
        if (options.AirVolume || options.Air)
            return Acore::StringFormat("map{:04}/{}_{}_air.n3t", options.MapId, tileX, tileY);
        return Acore::StringFormat("map{:04}/{}_{}_water.n3t", options.MapId, tileX, tileY);
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

    if (options.Probe)
        return ProbeMap(options) ? 0 : 1;
    if (options.AirTreeDetail)
        return GenerateAirTreeDetailTile(options) ? 0 : 1;

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

    std::ofstream manifest;
    if (!options.ManifestPath.empty())
    {
        manifest.open(options.ManifestPath, std::ios::app);
        if (!manifest)
        {
            std::cerr << "cannot open manifest " << options.ManifestPath << "\n";
            return 1;
        }
        manifest << "# map tileX tileY status\n";
        manifest.flush();
    }

    size_t failed = 0;
    for (TileRequest const& request : requests)
    {
        Options tileOptions = options;
        tileOptions.MapId = request.MapId;
        std::filesystem::path const expected =
            std::filesystem::path(tileOptions.OutputDir) /
            RelativeOutputPath(tileOptions, request.TileX, request.TileY);
        if (tileOptions.SkipExisting)
        {
            std::error_code existsEc;
            if (std::filesystem::exists(expected, existsEc))
            {
                std::cout << "skip existing " << expected << "\n";
                if (manifest)
                {
                    manifest << request.MapId << ' ' << request.TileX << ' ' << request.TileY
                             << " skipped\n";
                    manifest.flush();
                }
                continue;
            }
        }
        bool const generated = tileOptions.AirVolume
                                   ? GenerateAirVolumeTile(tileOptions, request.TileX, request.TileY)
                                   : tileOptions.Air
                                         ? GenerateTerrainAirTile(tileOptions, request.TileX, request.TileY)
                                         : tileOptions.TerrainOnly
                                               ? GenerateTerrainWaterTile(tileOptions, request.TileX,
                                                                          request.TileY)
                                               : GenerateTile(tileOptions, request.TileX, request.TileY);
        if (manifest)
        {
            manifest << request.MapId << ' ' << request.TileX << ' ' << request.TileY << ' '
                     << (generated ? "ok" : "failed") << '\n';
            manifest.flush();
        }
        if (!generated)
        {
            std::cerr << "failed map " << request.MapId << " tile " << request.TileX << ":" << request.TileY
                      << "\n";
            ++failed;
        }
    }

    return failed == 0 ? 0 : 1;
}
