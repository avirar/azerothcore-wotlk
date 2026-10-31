# Combo branch: travel nodes + POI Z + nav3d

This branch combines three stacks that are normally developed separately. It is
meant for testing and sharing, not as a PR.

## What is in the combo

| Layer | Source | Notes |
|---|---|---|
| POI Z / RPG stack | `test/z-fix` (`fa159277`) | Baked `playerbots_quest_poi_z` destinations, cross-zone questing, quest stats. |
| 3D navigation | `feat/nav3d-tiles` | Volumetric `.n3t` routing for swimming, WMO interiors and flight, cooperative multi-tick path search, Dalaran/Wintergrasp restricted flight. |
| Travel nodes | `celandriel/nav/enable_travel_node` (Keleborn/Celandriel) | Travel-node graph, `MoveTo2` / `DispatchMovement` funnel, transport boarding and waiting, prepath/spline reduction. |

The module branch is `combo/travel-poi-nav3d` in
`https://github.com/avirar/mod-playerbots.git`. The module is **not** a git
submodule in this repo (`modules/*` is ignored), so clone it separately:

```sh
git clone -b combo/travel-poi-nav3d https://github.com/avirar/mod-playerbots.git \
  modules/mod-playerbots
```

Core side: branch `combo/travel-poi-nav3d` in
`https://github.com/avirar/azerothcore-wotlk.git`, based on
`fix/escort-forced-fly-z` (`677058923`). It only adds this file; all combo code
is in the module branch.

## Data you must generate

Nothing generated is committed. The combo works without the optional data, but
the corresponding features stay disabled or fall back to 2D movement.

### 1. mmaps / vmaps

Generate them the usual AzerothCore way (`mmaps_generator`, `vmap4_extractor`,
`vmap4_assembler`).

### 2. nav3d tiles (`.n3t`)

Build `nav3d_generator` from this core and generate the tile tree, then point
`Playerbots.Nav3D.DataPath` at it (relative to the data directory):

```ini
Playerbots.Nav3D.Enabled = 1
Playerbots.Nav3D.DataPath = nav3d
```

The generator supports open-water tiles, air tiles and `--air-tree-detail`
boxes. Use `--manifest` to track which tiles were produced. Bake offline, never
on the worldserver.

### 3. travel-node graph

The schema ships in `data/sql/playerbots/base/playerbots_travelnode*.sql`.
Note that the stock base file already carries a full 2024 walk/taxi dataset
(~1.5M path points, 67 MB). The 2.8M-row travel-branch regeneration is not
committed. Regenerate the graph once on a GM console:

```
.playerbots travel generatenode
```

The command blocks the world thread until finished and saves to the
`playerbots_travelnode*` tables. Run it on a test realm, then back up the
tables. The generator refuses to run when
`Playerbots.EnableTravelNodes = 0`.

### 4. 3D travel paths (swim / free flight)

`playerbots_travelnode_link3d` and `playerbots_travelnode_path3d` are optional
pre-baked 3D routes. Build the standalone baker (no worldserver needed):

```sh
cmake -S . -B build -DTOOLS_BUILD=all
cmake --build build --target nav3d_travel_baker
```

Export nodes and bake:

```sh
mysql -N -e "SELECT id,map_id,x,y,z FROM playerbots_travelnode" acore_playerbots > nodes.txt
nav3d_travel_baker --nodes nodes.txt --tiles env/dist/bin/nav3d \
  --out bake_3d --medium both --threads 8 --radius 2000
```

It writes `travelnode_water.sql`, `travelnode_air.sql` and
`travelnode_bake.sql`. Import them into `acore_playerbots`. The baker uses only
the `.n3t` tiles (no Map, no live LOS), so run it offline and review the output
before importing. `--max-pairs N` is useful for a smoke test.

The runtime selects a 3D link only when the bot can use it: free flight while
flying, swim while swimming/underwater. Otherwise the 2D walk link is used.

### 5. quest POI Z

`QuestPoiZStore` reads `playerbots_quest_poi_z` from the playerbots database.
Bake it with `acore-data/scripts/quest_poi_z_baker.py`. When the table is
missing the store logs a message and quest destinations use the terrain-height
fallback.

## Configuration

```ini
# Travel-node graph (long overland journeys).
Playerbots.EnableTravelNodes = 1
Playerbots.TravelNodeProbeSteps = 10
Playerbots.TravelNodeDirectDistance = 300

# 3D navigation.
Playerbots.Nav3D.Enabled = 1
Playerbots.Nav3D.Cooperative = 1
Playerbots.Nav3D.PathTickBudgetMs = 1
Playerbots.Nav3D.PathGlobalBudgetMs = 120
Playerbots.Nav3D.PathMaxPerMap = 64
Playerbots.Nav3D.PathRetryMs = 2000
Playerbots.Nav3D.PathAgeBoostMs = 3000
Playerbots.Nav3D.TileCacheSize = 2048
```

`Playerbots.Nav3D.Enabled = 0` falls back to the classic 2D movers.
`Playerbots.EnableTravelNodes = 0` makes `MoveTo` skip the travel pipeline
entirely.

## How the layers cooperate

- `MovementAction::MoveTo` keeps the nav3d guards (dive surfacing, surface hold,
  pathless-swim clearance, `TryNav3DFly` / `TryNav3DMove`).
- Long normal-priority overland trips (`> TravelNodeDirectDistance` or a
  different map) are handed to the travel pipeline (`MoveTo2`), which can use
  the node graph, transports and flight paths.
- Combat, loot, swim/dive, flying, vehicle, exact-waypoint and short movement
  stay on the nav3d/2D path.
- `NewRpgBaseAction::MoveFarTo` uses the travel pipeline for cross-map and
  long-haul objectives, and the nav3d local steering for everything else.

## Known limitations

- Generated travel-node data is intentionally excluded from the branch; the
  first `.playerbots travel generatenode` run is required for graph routing.
- Travel A* has a fixed 2000-node budget; some long or badly connected routes
  log `[TravelFail] A* search budget exhausted` and fall back.
- 3D travel paths are generated data. The baker is tile-only; it does not
  validate against live VMAP LOS. Keep `--max-pairs`/small runs for review.
- Travel node ids are assigned by the node store. Rebake the 3D links after
  any `.playerbots travel generatenode` run; old ids may point at different
  nodes.
- `MaxCoreStuckTime` must be raised while running `.playerbots travel
  generatenode`; the command blocks the world thread for several minutes.
- GO teleport links now accept both `GAMEOBJECT_TYPE_SPELLCASTER` and
  `GAMEOBJECT_TYPE_GOOBER` (Orb of Translocation 184500/184501, Naxx orbs).
- Moving WMOs (gunships, transports) and Skybreaker/Orgrim's Hammer routing are
  not covered by the nav3d tiles.
- Dalaran/Wintergrasp restricted flight uses a timed one-way blocker with a
  parachute fallback; long transit through restricted airspace is rejected.
- Detail air tiles are optional. Tiles baked before the `--box` axis fix should
  be rebaked (`--box minX maxX minY maxY minZ maxZ`).
