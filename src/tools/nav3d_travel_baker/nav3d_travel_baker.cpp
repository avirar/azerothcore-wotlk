/*
 * Offline 3D travel-path baker.
 *
 * Reads travel nodes (id,map,x,y,z) and bakes swim / free-flight routes from
 * the nav3d .n3t tiles, emitting SQL for playerbots_travelnode_link3d and
 * playerbots_travelnode_path3d. It does not need a worldserver, a Map or a DB
 * connection; run it on a workstation or a build host.
 */

#include "Nav3DTile.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace
{
    constexpr float GRID = 533.3333f;
    constexpr float kWaterStep = 2.0f;
    constexpr float kAirStep = 10.0f;
    constexpr uint32 kMaxExpansions = 1000000;
    constexpr uint32 kMaxPoints = 8192;
    constexpr float kGridOffset = 0.37f;
    constexpr uint8 kSwimType = 7;
    constexpr uint8 kAirType = 8;
    bool gVerbose = false;

    struct Node
    {
        uint32 id = 0;
        uint32 map = 0;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct Point
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct Cell
    {
        int32 x = 0;
        int32 y = 0;
        int32 z = 0;

        bool operator==(Cell const& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct CellHash
    {
        size_t operator()(Cell const& c) const
        {
            uint64_t h = uint64_t(uint32_t(c.x)) * 73856093ull;
            h ^= uint64_t(uint32_t(c.y)) * 19349663ull;
            h ^= uint64_t(uint32_t(c.z)) * 83492791ull;
            return size_t(h);
        }
    };

    struct State
    {
        float g = 0.0f;
        float f = 0.0f;
        bool open = false;
        bool closed = false;
        Cell parent{};
        bool hasParent = false;
    };

    struct QueueEntry
    {
        float f = 0.0f;
        Cell cell{};
        bool operator>(QueueEntry const& other) const { return f > other.f; }
    };

    struct Options
    {
        std::string nodesPath;
        std::string tilesRoot;
        std::string outDir = "travel_bake";
        float radius = 2000.0f;
        float minDistance = 50.0f;
        float waterStep = kWaterStep;
        float airStep = kAirStep;
        uint32 threads = 4;
        uint32 maxPairs = 0;
        uint8 medium = 0;  // 0 = both, 1 = water, 2 = air
        bool probe = false;
        bool verbose = false;
    };

    std::string MediumName(uint8 type)
    {
        return type == kAirType ? "air" : "water";
    }

    uint64_t Fnv1a(std::string const& text, uint64_t seed = 1469598103934665603ull)
    {
        uint64_t hash = seed;
        for (unsigned char c : text)
        {
            hash ^= c;
            hash *= 1099511628211ull;
        }
        return hash;
    }

    bool LoadNodes(std::string const& path, std::vector<Node>& nodes)
    {
        std::ifstream in(path);
        if (!in)
        {
            std::cerr << "cannot open nodes file " << path << "\n";
            return false;
        }

        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream row(line);
            Node node;
            if (!(row >> node.id >> node.map >> node.x >> node.y >> node.z))
                continue;
            nodes.push_back(node);
        }
        return !nodes.empty();
    }

    uint32_t TileCoord(float value)
    {
        int32_t const tile = int32_t(std::floor(32.0f - value / GRID));
        return uint32_t(std::clamp<int32_t>(tile, 0, 63));
    }

    std::string TilePath(Options const& options, uint32 map, uint32 tx, uint32 ty, bool air)
    {
        char name[64];
        std::snprintf(name, sizeof(name), "map%04u/%02u_%02u_%s.n3t", map, tx, ty,
                      air ? "air" : "water");
        return (std::filesystem::path(options.tilesRoot) / name).string();
    }

    class TileCache
    {
    public:
        explicit TileCache(Options const& options) : _options(options) {}

        Nav3DTile const* Get(uint32 map, float x, float y, bool air)
        {
            uint32 const tx = TileCoord(x);
            uint32 const ty = TileCoord(y);
            uint64_t const key = (uint64_t(map) << 32) | (uint64_t(tx) << 16) | uint64_t(ty);
            auto const itr = _tiles.find(key);
            if (itr != _tiles.end())
                return itr->second.get();

            auto tile = std::make_unique<Nav3DTile>();
            if (!tile->Load(TilePath(_options, map, tx, ty, air)))
                return nullptr;
            Nav3DTile const* raw = tile.get();
            _tiles.emplace(key, std::move(tile));
            return raw;
        }

    private:
        Options const& _options;
        std::unordered_map<uint64_t, std::unique_ptr<Nav3DTile>> _tiles;
    };

    bool IsNavigable(TileCache& cache, uint32 map, float x, float y, float z, bool air)
    {
        Nav3DTile const* tile = cache.Get(map, x, y, air);
        if (!tile)
            return false;
        float const radius = air ? 0.8f : 0.5f;
        if (tile->IsNavigable(x, y, z, radius))
            return true;

        // ADT tile edges are not baked as navigable in both neighbours. Probe a
        // hair inside either side so a grid line exactly on x=0 (or y=0) does
        // not become an invisible wall.
        for (float const offset : {0.6f, -0.6f})
            if (cache.Get(map, x + offset, y, air) &&
                cache.Get(map, x + offset, y, air)->IsNavigable(x + offset, y, z, radius))
                return true;
        for (float const offset : {0.6f, -0.6f})
            if (cache.Get(map, x, y + offset, air) &&
                cache.Get(map, x, y + offset, air)->IsNavigable(x, y + offset, z, radius))
                return true;
        return false;
    }

    float Distance(Point const& a, Point const& b)
    {
        float const dx = a.x - b.x;
        float const dy = a.y - b.y;
        float const dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool SnapToNavigable(TileCache& cache, uint32 map, Point& point, bool air, float cell)
    {
        Point const base = point;
        float const zStart = air ? 0.0f : -30.0f;
        float const zEnd = air ? 45.0f : 40.0f;
        int const maxRadius = int(std::ceil(50.0f / std::max(cell, 0.5f)));
        for (int radius = 0; radius <= maxRadius; ++radius)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                for (int dy = -radius; dy <= radius; ++dy)
                {
                    if (radius != 0 && std::max(std::abs(dx), std::abs(dy)) != radius)
                        continue;
                    for (float dz = zStart; dz <= zEnd; dz += air ? 2.0f : 1.5f)
                    {
                        Point const candidate{base.x + dx * cell, base.y + dy * cell, base.z + dz};
                        if (IsNavigable(cache, map, candidate.x, candidate.y, candidate.z, air))
                        {
                            point = candidate;
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    struct Bounds
    {
        float minX, minY, minZ, maxX, maxY, maxZ;
    };

    bool Inside(Bounds const& b, Point const& p)
    {
        return p.x >= b.minX && p.x <= b.maxX && p.y >= b.minY && p.y <= b.maxY &&
               p.z >= b.minZ && p.z <= b.maxZ;
    }

    // Tile-only A*. The offline baker does not have a Map, so the search uses
    // the baked navigability/clearance intervals and samples every edge.
    bool FindPath(TileCache& cache, Node const& start, Node const& goal, bool air, float step,
                  std::vector<Point>& out)
    {
        Point startPoint{start.x, start.y, start.z};
        Point goalPoint{goal.x, goal.y, goal.z};
        float const snapStep = air ? std::max(step, 2.0f) : std::max(1.0f, step * 0.5f);
        if (!SnapToNavigable(cache, start.map, startPoint, air, snapStep) ||
            !SnapToNavigable(cache, start.map, goalPoint, air, snapStep))
        {
            if (gVerbose)
                std::cerr << "snap failed " << start.id << "->" << goal.id << " air=" << air
                          << " start=(" << startPoint.x << "," << startPoint.y << "," << startPoint.z
                          << ") goal=(" << goalPoint.x << "," << goalPoint.y << "," << goalPoint.z
                          << ")\n";
            return false;
        }

        float const direct = Distance(startPoint, goalPoint);
        if (direct < 1.0f)
            return false;

        float const marginXY = std::max(150.0f, direct * 0.3f);
        // Do not inflate Z with the horizontal length: a 1,500 yd route would
        // otherwise search a 1,000 yd tall box. Air starts and ends at ground
        // level, so the vertical detour is bounded.
        float const marginZ = air
            ? std::max(250.0f, std::fabs(startPoint.z - goalPoint.z) + 150.0f)
            : std::max(60.0f, std::fabs(startPoint.z - goalPoint.z) + 80.0f);
        Bounds const bounds{std::min(startPoint.x, goalPoint.x) - marginXY,
                            std::min(startPoint.y, goalPoint.y) - marginXY,
                            std::min(startPoint.z, goalPoint.z) - marginZ,
                            std::max(startPoint.x, goalPoint.x) + marginXY,
                            std::max(startPoint.y, goalPoint.y) + marginXY,
                            std::max(startPoint.z, goalPoint.z) + marginZ};

        auto cellOf = [step](Point const& p)
        {
            return Cell{int32_t(std::llround((p.x - kGridOffset) / step)),
                        int32_t(std::llround((p.y - kGridOffset) / step)),
                        int32_t(std::llround((p.z - kGridOffset) / step))};
        };
        auto pointOf = [step](Cell const& c)
        {
            return Point{float(c.x) * step + kGridOffset, float(c.y) * step + kGridOffset,
                         float(c.z) * step + kGridOffset};
        };

        auto cellNavigable = [&](Cell const& c)
        {
            Point const p = pointOf(c);
            return IsNavigable(cache, start.map, p.x, p.y, p.z, air);
        };
        auto snapCell = [&](Cell& cell)
        {
            if (cellNavigable(cell))
                return true;
            for (int radius = 1; radius <= 3; ++radius)
            {
                for (int dx = -radius; dx <= radius; ++dx)
                {
                    for (int dy = -radius; dy <= radius; ++dy)
                    {
                        for (int dz = -radius; dz <= radius; ++dz)
                        {
                            Cell const candidate{cell.x + dx, cell.y + dy, cell.z + dz};
                            if (cellNavigable(candidate))
                            {
                                cell = candidate;
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        };

        Cell startCell = cellOf(startPoint);
        Cell goalCell = cellOf(goalPoint);
        if (!snapCell(startCell) || !snapCell(goalCell))
        {
            if (gVerbose)
                std::cerr << "cell snap failed " << start.id << "->" << goal.id << " air=" << air << "\n";
            return false;
        }
        startPoint = pointOf(startCell);
        goalPoint = pointOf(goalCell);
        if (startCell == goalCell)
            return false;

        auto heuristic = [&](Cell const& c)
        {
            return Distance(pointOf(c), goalPoint);
        };

        std::unordered_map<Cell, State, CellHash> states;
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
        State& startState = states[startCell];
        startState.f = heuristic(startCell);
        startState.open = true;
        open.push({startState.f, startCell});

        uint32 expansions = 0;
        bool found = false;

        while (!open.empty() && expansions++ < kMaxExpansions)
        {
            QueueEntry const entry = open.top();
            open.pop();
            auto itr = states.find(entry.cell);
            if (itr == states.end() || itr->second.closed || entry.f > itr->second.f)
                continue;
            State& current = itr->second;
            current.open = false;
            current.closed = true;

            if (entry.cell == goalCell)
            {
                found = true;
                break;
            }

            // Copy before inserting into the map; rehashing invalidates `current`.
            float const currentG = current.g;
            Cell const currentCell = entry.cell;

            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dz = -1; dz <= 1; ++dz)
                    {
                        if (!dx && !dy && !dz)
                            continue;
                        Cell const next{currentCell.x + dx, currentCell.y + dy, currentCell.z + dz};
                        Point const p = pointOf(next);
                        if (!Inside(bounds, p))
                            continue;
                        if (!IsNavigable(cache, start.map, p.x, p.y, p.z, air))
                            continue;

                        // Do not cut through the corner of a solid cell.
                        if (dx && dy)
                        {
                            Point const mid{float(currentCell.x + dx) * step,
                                            float(currentCell.y + dy) * step,
                                            float(currentCell.z) * step};
                            if (!IsNavigable(cache, start.map, mid.x, mid.y, mid.z, air))
                                continue;
                        }

                        Point const from = pointOf(currentCell);
                        float const move = Distance(from, p);
                        float const tentative = currentG + move * (1.0f + (dz ? 0.03f : 0.0f));
                        State& nextState = states[next];
                        if (nextState.closed || (nextState.open && tentative >= nextState.g))
                            continue;
                        nextState.g = tentative;
                        nextState.f = tentative + heuristic(next);
                        nextState.open = true;
                        nextState.closed = false;
                        nextState.parent = currentCell;
                        nextState.hasParent = true;
                        open.push({nextState.f, next});
                    }
                }
            }
        }

        if (!found)
        {
            if (gVerbose)
            {
                auto const git = states.find(goalCell);
                std::cerr << "astar failed " << start.id << "->" << goal.id << " air=" << air
                          << " expansions=" << expansions << " states=" << states.size()
                          << " startCell=(" << startCell.x << "," << startCell.y << "," << startCell.z
                          << ") goalCell=(" << goalCell.x << "," << goalCell.y << "," << goalCell.z
                          << ") goalState=";
                if (git == states.end())
                    std::cerr << "absent";
                else
                    std::cerr << "g=" << git->second.g << " f=" << git->second.f
                              << " open=" << git->second.open << " closed=" << git->second.closed;
                std::cerr << "\n";
            }
            return false;
        }

        std::vector<Cell> cells;
        Cell cursor = goalCell;
        while (true)
        {
            cells.push_back(cursor);
            auto const itr = states.find(cursor);
            if (itr == states.end() || !itr->second.hasParent)
                break;
            cursor = itr->second.parent;
            if (cells.size() > kMaxPoints)
                return false;
        }
        std::reverse(cells.begin(), cells.end());

        // Keep only turn points so long straights stay compact.
        std::vector<Point> points;
        for (Cell const& cell : cells)
        {
            Point const p = pointOf(cell);
            if (points.size() >= 2)
            {
                Point const& a = points[points.size() - 2];
                Point const& b = points.back();
                Point const ab{b.x - a.x, b.y - a.y, b.z - a.z};
                Point const bc{p.x - b.x, p.y - b.y, p.z - b.z};
                float const crossX = ab.y * bc.z - ab.z * bc.y;
                float const crossY = ab.z * bc.x - ab.x * bc.z;
                float const crossZ = ab.x * bc.y - ab.y * bc.x;
                float const cross = std::sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);
                float const len = std::sqrt(ab.x * ab.x + ab.y * ab.y + ab.z * ab.z) *
                                  std::sqrt(bc.x * bc.x + bc.y * bc.y + bc.z * bc.z);
                if (len > 0.001f && cross / len < 0.08f)
                    points.back() = p;
                else
                    points.push_back(p);
            }
            else
                points.push_back(p);
        }
        if (points.size() < 2)
            return false;
        points.front() = startPoint;
        points.back() = goalPoint;
        out = std::move(points);
        return out.size() >= 2;
    }

    struct Result
    {
        uint8 type = 0;
        uint32 from = 0;
        uint32 to = 0;
        uint32 map = 0;
        float distance = 0.0f;
        float extraCost = 0.0f;
        std::vector<Point> points;
    };

    float PathLength(std::vector<Point> const& points)
    {
        float total = 0.0f;
        for (size_t i = 1; i < points.size(); ++i)
            total += Distance(points[i - 1], points[i]);
        return total;
    }

    void WriteSql(std::string const& path, std::vector<Result> const& results, uint8 type)
    {
        std::ofstream out(path);
        if (!out)
        {
            std::cerr << "cannot write " << path << "\n";
            return;
        }

        out << "-- Generated by nav3d_travel_baker\n";
        out << "DELETE FROM `playerbots_travelnode_link3d` WHERE `type`=" << uint32(type) << ";\n";
        out << "DELETE FROM `playerbots_travelnode_path3d` WHERE `type`=" << uint32(type) << ";\n";

        constexpr size_t kBatch = 500;
        for (size_t offset = 0; offset < results.size(); offset += kBatch)
        {
            size_t const end = std::min(offset + kBatch, results.size());
            out << "INSERT INTO `playerbots_travelnode_link3d` "
                   "(`node_id`,`to_node_id`,`type`,`distance`,`swim_distance`,`extra_cost`,`calculated`) VALUES\n";
            for (size_t i = offset; i < end; ++i)
            {
                Result const& r = results[i];
                out << "(" << r.from << "," << r.to << "," << uint32(type) << "," << r.distance << ","
                    << (type == kSwimType ? r.distance : 0.0f) << "," << r.extraCost << ",1)"
                    << (i + 1 < end ? "," : ";") << "\n";
            }
        }

        size_t row = 0;
        for (size_t offset = 0; offset < results.size(); offset += kBatch)
        {
            size_t const end = std::min(offset + kBatch, results.size());
            out << "INSERT INTO `playerbots_travelnode_path3d` "
                   "(`node_id`,`to_node_id`,`type`,`nr`,`map_id`,`x`,`y`,`z`) VALUES\n";
            bool first = true;
            for (size_t i = offset; i < end; ++i)
            {
                Result const& r = results[i];
                for (size_t n = 0; n < r.points.size(); ++n, ++row)
                {
                    if (!first)
                        out << ",\n";
                    first = false;
                    out << "(" << r.from << "," << r.to << "," << uint32(type) << "," << n << ","
                        << r.map << "," << r.points[n].x << "," << r.points[n].y << ","
                        << r.points[n].z << ")";
                }
            }
            out << ";\n";
        }
    }
}

int main(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        std::string const arg = argv[i];
        auto next = [&](std::string& value) { if (i + 1 < argc) value = argv[++i]; };
        if (arg == "--nodes") next(options.nodesPath);
        else if (arg == "--tiles") next(options.tilesRoot);
        else if (arg == "--out") next(options.outDir);
        else if (arg == "--threads") options.threads = std::max(1, std::atoi(argv[++i]));
        else if (arg == "--radius") options.radius = float(std::atof(argv[++i]));
        else if (arg == "--min-distance") options.minDistance = float(std::atof(argv[++i]));
        else if (arg == "--step-water") options.waterStep = float(std::atof(argv[++i]));
        else if (arg == "--step-air") options.airStep = float(std::atof(argv[++i]));
        else if (arg == "--max-pairs") options.maxPairs = uint32(std::atoi(argv[++i]));
        else if (arg == "--medium")
        {
            std::string medium;
            next(medium);
            options.medium = medium == "water" ? 1 : medium == "air" ? 2 : 0;
        }
        else if (arg == "--probe") options.probe = true;
        else if (arg == "--verbose") options.verbose = true;
        else if (arg == "--help")
        {
            std::cout << "usage: nav3d_travel_baker --nodes nodes.csv --tiles nav3d --out dir "
                         "[--medium water|air|both] [--threads N] [--radius 2000] "
                         "[--min-distance 50] [--step-water 5] [--step-air 10] [--max-pairs N]\n";
            return 0;
        }
    }

    if (options.probe)
    {
        if (options.nodesPath.empty() || options.tilesRoot.empty())
        {
            std::cerr << "--nodes and --tiles are required for --probe\n";
            return 1;
        }
        std::vector<Node> probeNodes;
        if (!LoadNodes(options.nodesPath, probeNodes))
            return 1;
        TileCache cache(options);
        for (Node const& node : probeNodes)
        {
            bool const air = options.medium != 1;
            Nav3DTile const* tile = cache.Get(node.map, node.x, node.y, air);
            std::cout << "node " << node.id << " map " << node.map << " (" << node.x << ","
                      << node.y << "," << node.z << ") tile=" << (tile ? "yes" : "missing");
            if (tile)
            {
                Nav3DTileHeader const& h = tile->Header();
                std::cout << " header=(" << h.OriginX << "," << h.OriginY << "," << h.OriginZ
                          << " size=" << h.SizeX << "x" << h.SizeY << "x" << h.SizeZ
                          << " cell=" << h.CellSize << " medium=" << uint32(h.Medium) << ")";
                std::cout << " nav=" << (tile->IsNavigable(node.x, node.y, node.z, 0.8f) ? 1 : 0);
            }
            std::cout << "\n";
        }
        return 0;
    }

    if (options.nodesPath.empty() || options.tilesRoot.empty())
    {
        std::cerr << "--nodes and --tiles are required\n";
        return 1;
    }

    gVerbose = options.verbose;

    std::vector<Node> nodes;
    if (!LoadNodes(options.nodesPath, nodes))
        return 1;

    std::filesystem::create_directories(options.outDir);

    std::unordered_map<uint32, std::vector<Node>> byMap;
    for (Node const& node : nodes)
        byMap[node.map].push_back(node);

    bool const doWater = options.medium == 0 || options.medium == 1;
    bool const doAir = options.medium == 0 || options.medium == 2;
    uint64_t const nodeHash = Fnv1a(std::to_string(nodes.size()));

    std::cout << "loaded " << nodes.size() << " nodes on " << byMap.size() << " maps\n";

    auto runMedium = [&](uint8 type, bool air, float step)
    {
        std::vector<Result> results;
        std::mutex resultMutex;
        std::atomic<uint64_t> done{0};
        std::atomic<uint64_t> failed{0};
        std::atomic<bool> limitReached{false};

        std::vector<std::thread> workers;
        for (uint32 t = 0; t < options.threads; ++t)
        {
            workers.emplace_back([&, t]()
            {
                TileCache cache(options);
                for (auto const& [map, mapNodes] : byMap)
                {
                    for (size_t a = t; a < mapNodes.size(); a += options.threads)
                    {
                        for (size_t b = 0; b < mapNodes.size(); ++b)
                        {
                            if (a == b)
                                continue;
                            if (options.maxPairs > 0 && done.load() >= options.maxPairs)
                            {
                                limitReached = true;
                                break;
                            }
                            Node const& start = mapNodes[a];
                            Node const& goal = mapNodes[b];
                            float const dist = Distance(Point{start.x, start.y, start.z},
                                                        Point{goal.x, goal.y, goal.z});
                            if (dist < options.minDistance || dist > options.radius)
                                continue;

                            std::vector<Point> points;
                            if (FindPath(cache, start, goal, air, step, points))
                            {
                                Result r;
                                r.type = type;
                                r.from = start.id;
                                r.to = goal.id;
                                r.map = map;
                                r.distance = PathLength(points);
                                r.extraCost = r.distance / (air ? 20.0f : 4.0f);
                                r.points = std::move(points);
                                std::lock_guard<std::mutex> guard(resultMutex);
                                results.push_back(std::move(r));
                            }
                            else
                                ++failed;
                            ++done;
                        }
                        if (limitReached)
                            break;
                    }
                    if (limitReached)
                        break;
                }
            });
        }
        for (auto& worker : workers)
            worker.join();

        std::sort(results.begin(), results.end(), [](Result const& a, Result const& b)
        {
            return a.from != b.from ? a.from < b.from : a.to < b.to;
        });

        std::string const name = options.outDir + "/travelnode_" + MediumName(type) + ".sql";
        WriteSql(name, results, type);
        std::cout << MediumName(type) << ": stored " << results.size() << " links, failed "
                  << failed.load() << " pairs -> " << name << "\n";
    };

    if (doWater)
        runMedium(kSwimType, false, options.waterStep);
    if (doAir)
        runMedium(kAirType, true, options.airStep);

    std::ofstream meta(options.outDir + "/travelnode_bake.sql");
    meta << "INSERT INTO `playerbots_travelnode_bake` "
            "(`id`,`nav3d_hash`,`mmap_hash`,`node_hash`,`baker_version`) VALUES\n";
    bool first = true;
    auto writeMeta = [&](uint8 id)
    {
        if (!first)
            meta << ",\n";
        first = false;
        meta << "(" << uint32(id) << ",0,0," << nodeHash << ",'nav3d_travel_baker')";
    };
    if (doWater)
        writeMeta(kSwimType);
    if (doAir)
        writeMeta(kAirType);
    meta << ";\n";
    meta.close();

    return 0;
}
