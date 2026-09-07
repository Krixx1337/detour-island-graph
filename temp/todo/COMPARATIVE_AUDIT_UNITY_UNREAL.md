# DetourIslandGraph — Comparative Audit vs Unity Auto Off-Mesh Links and Unreal Automatic Nav-Link Generation

Date: 2026-09-07
Scope: `DetourIslandGraph` link discovery, validation, filtering, tuning, performance, and robustness —
compared against Unity automatic Off-Mesh Links and Unreal Engine automatic Navigation Link generation
(docs + `E:\Projects\UnrealEngine-release` source). Implementation-focused. Proven techniques are separated
from speculation in every recommendation.

Existing internal audit: `temp/AUDIT.md` (status table + findings 1–9, performance notes, third-party assessment).
This file does not repeat that audit; it cites it where the external comparison confirms, contradicts, or extends it.

## 1. Baseline: what DetourIslandGraph actually does today

Pipeline (`src/IslandGraphBuilder.cpp`, `src/IslandGraphDiscovery.cpp`, `src/IslandGraphTopology.cpp`,
`src/IslandGraphBoundaries.cpp`, `src/IslandGraphScanner.cpp`, `src/IslandGraphPruning.cpp`):

1. `floodFill`: tile iteration over `dtNavMesh`, BFS through internal + all external `dtLink`s into islands
   of eligible polygons (`DT_POLYTYPE_GROUND` by default + Detour include/exclude + optional predicate).
2. `calculateMassScores`: optional mass = `polyCount * dominantSpan`, log-normalized at p99; optional
   bottom-percent suppression (activates at >= 20 islands).
3. `extractBoundaries`: every boundary edge (no linked neighbor) becomes a `Boundary` with midpoint;
   optional voxel dedup keyed `(island, quantized midpoint XYZ, quantized direction XZ)` with a separate,
   smaller vertical collapse window.
4. `selectBoundaryRepresentatives`: optional reduction to best-per-`(island, cell, direction-bucket)` cell
   (longest edge wins by default, custom ranker allowed) + optional per-island mass/complexity budgets with
   spatially-stratified slot selection.
5. `discoverCandidates`: for each representative midpoint, `queryPolygons(center, extents={hGap, vExtent, hGap})`
   + `closestPointOnPoly` per nearby polygon on another island; accept when
   `horizontal <= maxHorizontalGap && up/down within limits`. Optional pair-scan suppression (symmetric limits
   only) keeps best-per-`(islandPair, midpointCell)`; optional short-gap recovery re-scans all boundaries at a
   shorter range. 3D voxel candidate dedup preserves direction classes.
6. `pruneCandidates`: sort by `isBetterLink` (mass importance, then rank = distance − target preference,
   mm-quantized, then deterministic tiebreaks); same-target local pruning; optional global endpoint pruning;
   optional 2-hop spanner pruning with Euclidean transfers and distinct-target reserve; exact-geometry edge
   assembly into symmetric-first `Edge{islandA, islandB, pointA, pointB, traversableAB, traversableBA}`;
   `calculateDirectionValidity` re-validates stored flags against limits + outbound policy.
7. Query: portal-level A* over traversals (`IslandGraphPathfinder.cpp`), default geometric cost, custom
   cost/filter/heuristic hooks, expansion/queue budgets, cancellation.

Strengths to preserve: the symmetric-first single-corridor model, direction re-validation at assembly,
same-target-only local pruning, spanner transfers, mass-aware budgets, and the unusually thorough
`BuildStats`/`DirectionValidityStats`/reachability/unreachable-component diagnostics are all ahead of what
either engine ships for link generation. Neither engine builds an island meta-graph or a high-level router;
they emit local off-mesh connections and leave topology to the pathfinder.

Core weakness confirmed by both engines: IslandGraph validates a zero-width straight segment between two
points and never checks (a) whether a physical trajectory between them is obstructed, (b) whether an agent
volume fits at takeoff, in flight, and at landing, (c) whether landing ground actually exists within tolerance
as opposed to the nearest polygon projecting nearby, or (d) anything except boundary midpoints. Everything
below follows from that gap.

## 2. Unity automatic Off-Mesh Links — what was actually learned

Source: https://docs.unity.cn/Manual/nav-BuildingOffMeshLinksAutomatically.html (Unity 2022.3 manual).

- Two link classes only: **Drop-Down** (controlled by `Drop Height`) and **Jump-Across** (controlled by
  `Jump Distance`). Setting either to 0 disables that class.
- Method: "the build process walks along the edges of the NavMesh and checks if the landing location of the
  jump is on NavMesh. If the jump trajectory is unobstructed an OffMesh link is created."
- Drop-Down trajectory: horizontal travel `A = 2*agentRadius + 4*voxelSize`, i.e. land just beyond the platform
  lip with voxel padding so voxelization rounding does not veto the link. Vertical travel `B` must exceed bake
  `Step Height` (else it is walkable, not a link) and be below `Drop Height`.
- Jump-Across trajectory: horizontal `C` in `(2*agentRadius, Jump Distance)`; landing `D` must be within
  `voxelSize` of start height — effectively same-level jumps only.
- Authoring: per-object **Generate OffMeshLinks** flag marks valid takeoff geometry; old links are discarded
  on every re-bake.
- Troubleshooting guidance is margin guidance: "set Drop Height a bit larger / Jump Distance a bit longer than
  measured," because bake deviation otherwise silently drops links. Jump Distance is measured NavMesh-to-NavMesh,
  so users must add `2*agentRadius` plus margin.

Proven ideas worth adopting: (1) walk the edge, not just its midpoint; (2) obstructed-trajectory rejection;
(3) agent-radius + voxel Terms in the takeoff/landing placement so rasterization noise does not decide
connectivity; (4) explicit "below step height is walking, not linking" lower bound; (5) margin/tolerance as a
first-class tuning concept rather than exact-limit equality.

What not to adopt: the Jump-Across same-level (`voxelSize`) restriction is too narrow for IslandGraph's
cliffs/rooftops/stacked-layers use case, and Unity's two-class model cannot express asymmetric climb/drop
capabilities or one-way routes. Unity also scopes generation to flagged objects at bake time; IslandGraph's
post-build global pass with outbound-island policy is strictly more flexible — keep it.

## 3. Unreal automatic Navigation Link generation — docs

Source: https://dev.epicgames.com/documentation/unreal-engine/automatic-navigation-link-generation?lang=en-US
(UE 5.5+ Experimental, 5.8 docs).

Config surface (`FNavLinkGenerationJumpConfig`, formerly `...JumpDownConfig`):
`JumpLength` (horizontal reach), `JumpDistanceFromEdge` (takeoff set back from edge), `JumpMaxDepth`
(how far below start to look; negative + large `JumpHeight` = upward landing), `JumpHeight` (parabola peak),
`JumpEndsHeightTolerance` (ground-search tolerance at both ends), `SamplingSeparationFactor` (× cell size =
distance between trajectory samples along the edge), `FilterDistanceThreshold` (similar-link suppression;
0 = off), `LinkBuilderFlags` (center-point vs extremity emission), per-direction area classes
(`DownDirectionAreaClass` / `UpDirectionAreaClass`; equal = one bidirectional link, different = two directed
links, null = suppress that direction), `LinkProxyClass` (`GeneratedNavLinksProxy` for custom traversal
behavior). Multiple configs can coexist in `NavLinkJumpConfigs`.

Performance model stated in the docs: cost is per border edge per tile; `JumpLength` dominates because it
expands rasterization (`LinkSpillDistance` → tile border, see §4); `SamplingSeparationFactor` trades sampling
precision against speed; per-tile link build time is observable via `Draw Tile Build Times`.

Proven ideas worth adopting: takeoff offset, endpoint ground tolerance, sampling density rooted in cell size,
segment-distance dedup with a zero-off switch, center vs extremity emission, per-direction area/flag split,
multi-profile configs in one build, behavior binding via proxy class, and build-time telemetry.

## 4. Unreal source — `dtNavLinkBuilder` mechanics actually inspected

All paths under `E:\Projects\UnrealEngine-release`. Line numbers are approximate (release branch moves).

### 4.1 Edge extraction (`Engine/Source/Runtime/Navmesh/Private/Detour/DetourNavLinkBuilder.cpp`, `findEdges`, ~L64–160)

- Source geometry is the tile-cache contour set, not the final polys. Portal edges
  (`va[3] & 0xf != 0xf`) are skipped; edges with a matching reversed contour elsewhere are treated as shared
  internal edges and skipped. Survivors are stored in world coordinates with a `+2*ch` height bias.
- Contrast with IslandGraph: `IslandGraphBoundaries.cpp::isBoundaryEdge` treats an external edge with zero
  resolved `dtLink`s as boundary and any linked external edge as fully internal. The internal `AUDIT.md`
  finding 9 (partial-portal coverage, tile-order dependence) is the same class of issue UE sidesteps by
  working pre-contour-match at voxel scale. Proven takeaway: contour/portal-aware boundary classification is
  more robust than link-presence classification; at minimum, uncovered intervals of partially linked portals
  should stay boundaries.

### 4.2 Jump rig and trajectory (`initJumpRig` ~L670–721, `DetourNavLinkBuilderConfig.cpp::init`)

- Parabola `y(x) = ax² + (−d/l − al)x` with cached `a` solved from (`jumpLength`, `jumpMaxDepth`, `jumpHeight`)
  and cached `downRatio = −d/l`. The spine has `MAX_SPINE = 8` samples from `−jumpDistanceFromEdge` to
  `jumpLength − jumpDistanceFromEdge`. Edge tips are trimmed by one cell size along long edges to avoid
  raster-border height errors; short edges keep original endpoints.
- Contrast: IslandGraph has no trajectory at all — acceptance is `hypot(dx,dz) <= hGap && dy in limits`.
  Any heightfield obstacle between the points is invisible. This is the single largest robustness delta in the
  comparison and it is proven, not speculative: both engines trajectory-check, IslandGraph does not.

### 4.3 Ground search and clearance (`sampleGroundSegment`, `getCompactHeightfieldHeight`,
### `updateTrajectorySamples`, `sampleAction`, `isTrajectoryClear`, `checkHeightfieldCollision`, ~L378–610)

- Start/end ground segments are offset along the trajectory normal, then sampled at
  `ngsamples = max(2, ceil(edgeLen / (separationFactor * cs)))` points; each sample looks up compact-heightfield
  ground within `jumpEndsHeightTolerance`.
- Trajectory samples are precomputed with agent-volume padding (`ymin += agentClimb`, `ymax += agentHeight`,
  `radiusOverflow = agentRadius`), then floored to sampled ground at both ends; each trajectory slice is tested
  against the solid heightfield (`checkHeightfieldCollision` over `[p.y+ymin, p.y+ymax]`).
- Per-edge-lateral-sample pass marks ground samples `UNRESTRICTED` only where the full trajectory is clear.
- Contrast: IslandGraph's `closestPointOnPoly` projection answers "nearest polygon point," not "standable
  ground within tolerance with headroom and a clear flight." Adopting at least a tolerance + clearance + walkability
  (area/flag) check at both endpoints would remove a whole class of false-positive links (thin rails, ledges
  with no headroom, projections onto polygons the agent filter would reject).

### 4.4 Link emission (`addEdgeLinks` ~L162–279)

- Median filter (`RAD = 2`) over `UNRESTRICTED` flags along the edge; contiguous runs become candidate spans;
  spans at edge ends get `+agentRadius` free-width credit; spans narrower than `agentRadius` are dropped.
  Each surviving span emits one `JumpLink` quad (`spine0`/`spine1` = span extremes swept along the trajectory,
  lifted by `agentClimb`).
- Contrast: IslandGraph emits at most one candidate per boundary-midpoint × nearby-polygon pair, then voxel-dedups.
  UE's span model discovers multiple usable intervals along one long edge and suppresses sub-agent-width slivers —
  directly addressing internal finding 8 (midpoint-only discovery is tessellation-sensitive). Proven pattern;
  IslandGraph can approximate it by sampling long boundary edges at `separation × resolution` intervals instead of
  midpoints only.

### 4.5 Redundant-link filtering (`filterOverlappingLinks` ~L281–341)

- O(n²) pairwise test: link J is redundant vs I iff all four endpoints are within threshold when measured as
  point-to-segment distances against the other link's start/end segments; keeps the wider link; skips pairs
  with different up/down areas so distinct traversal classes never collapse.
- Contrast: IslandGraph's candidate dedup voxel-hashes quantized endpoints + direction bits, and local pruning
  compares start/start + end/end within a radius for the same target. UE's segment-distance test handles
  near-parallel offset links better than endpoint voxels; its area-aware exemption is the exact analogue of
  IslandGraph's direction-class preservation (already fixed per `AUDIT.md` finding 4 — keep that invariant
  under any new filter). Note UE's O(n²) is fine per-tile but would not scale to IslandGraph's global candidate
  list; a spatial index is needed (speculative until benchmarked — cf. internal audit's nanoflann note).

### 4.6 Link materialization (`RecastNavMeshGenerator.cpp::AddGeneratedLinks` ~L3876–4017)

- `CreateCenterPointLink` emits the span-center line; `CreateExtremityLinks` emits both span edges.
  Equal up/down areas → one bidirectional off-mesh connection; different → two directed connections sharing
  endpoints with `LeftToRight`/`RightToLeft` semantics; null area suppresses that direction. Tile ownership is
  decided by XZ containment with direction flip so the owner endpoint is always local; `agentClimb` lift is
  subtracted back out for final heights.
- IslandGraph's symmetric-first `Edge` with independent `traversableAB/BA` flags already expresses all of this
  more compactly — do not regress to duplicated geometry. What is missing is extremity emission (useful for very
  wide spans) and per-direction area/flag metadata (useful for cost/filtering at query time). Both are proven,
  both are optional.

### 4.7 Tile/border accounting (`ComputeConfigBorderSizes` ~L3682–3707, config propagation ~L5334–5358,
### `BuildTileCacheLinks` ~L4019–4142)

- `LinkSpillDistance = max(JumpLength − JumpDistanceFromEdge)` expands tile borders (with a warning when the
  border exceeds tile size); per-config `init()` precomputes parabola constants once; generation loops
  configs × edges with per-config `buildForAllEdges` + `filterOverlappingLinks`; link build time is logged
  per tile (`BuildTileCacheLinks time: %0.3fms`).
- IslandGraph builds globally after the navmesh exists, so spill/borders do not apply directly. The portable
  lesson is per-config precomputation, config-major looping (which maps to multi-profile builds), and
  per-stage timing — IslandGraph already has the latter (`TimingStats`); add trajectory-sample and
  projection counters alongside `closestPointQueryCount` when new stages land.

## 5. Recommendations, ordered by value

Each item states the source, what changes, and whether it is proven or speculative.

### R1. Add trajectory + clearance validation to candidate acceptance (PROVEN — adopt first)

- Sources: UE `isTrajectoryClear`/`initTrajectorySamples`/`sampleAction`; Unity "if the jump trajectory is
  unobstructed an OffMesh link is created."
- Problem: `withinTraversalLimits` + `closestPointOnPoly` cannot see walls, low ceilings, or narrow posts
  between the points. Internal finding 6 already notes Euclidean transfers understate travel; the discovery
  side has the same flaw at larger magnitude.
- Change: introduce an optional `TrajectoryValidator(start, end, config) -> bool` consulted in
  `IslandGraphScanner.cpp::evaluateCandidate` after the limits check. Ship one built-in validator implementing
  a UE-style parabola (reuse `jumpHeight`-equivalent apex + `agentRadius/Height/Climb` padding) sampled
  against either the `dtNavMeshQuery` (raycast/`findStraightPath` obstruction probe — coarse) or, where the
  heightfield is available at build time, the solid/compact heightfields (precise, UE-proven). Default off
  until benchmarked; when off, behavior is today's.
- Why first: every downstream stage (dedup, pruning, spanner, stats) currently trusts unvalidated geometry.
  Validation at the source improves precision more than any additional pruning heuristic.

### R2. Sample along boundary edges, not just midpoints (PROVEN — adopt second)

- Sources: UE `ngsamples = max(2, ceil(edgeLen/(separation*cs)))` + span emission; Unity "walks along the edges."
- Problem: internal finding 8 — long edges whose midpoints are out of range but whose ends are in range are
  missed; tessellation changes connectivity. Short-gap recovery re-scans midpoints, compounding the bias.
- Change: in `IslandGraphBoundaries.cpp`, split boundary edges longer than `k * effectiveCellSize` into
  sub-segments (cap count per edge for telemetry), each with its own midpoint + parent edge reference; carry a
  `subSegmentIndex` through `Boundary` so dedup keys stay stable. Alternatively sample N points along the edge
  in the scanner directly. Either resolves finding 8 without changing the query path.
- Speculative part (only): the exact split factor. Start with `separation × deduplicationCellSize`, expose as
  `boundary.edgeSplitRatio`, benchmark link-count vs recall on a stacked-layer map.

### R3. Takeoff offset + endpoint ground tolerance (PROVEN — cheap, adopt with R2)

- Sources: UE `JumpDistanceFromEdge` + `JumpEndsHeightTolerance` + edge-tip trimming; Unity
  `2*agentRadius + 4*voxelSize` landing offset and "set limits a bit larger than measured" margin doctrine.
- Problem: IslandGraph starts links exactly at boundary midpoints and accepts any `closestPointOnPoly`
  projection within limits — no standable-ground check, no margin, no lip clearance. Boundary-adjacent
  projections produce knife-edge links that agents cannot execute.
- Change: add `gapDiscovery.takeoffOffset` (default 0 = today's behavior) applied along the outward edge
  normal... or, if normals are unavailable, along the candidate direction; add `endpointTolerance` applied as
  a ground-search window around each projected endpoint (accept only if a walkable polygon sample exists within
  the window with headroom per query filter). Document both as margins, Unity-style, with "measure then add
  margin" guidance in `README.md`.

### R4. Replace endpoint-voxel dedup with segment-distance suppression, keeping direction classes (PROVEN
### pattern, IslandGraph-scale indexing SPECULATIVE)

- Sources: UE `filterOverlappingLinks` (4-point point-to-segment test, keep-wider, area-aware skip).
- Problem: quantized-endpoint voxel keys merge near-parallel offset corridors unpredictably under translation,
  and split identical corridors across voxel boundaries. Internal finding 7 already flags absolute-scale
  sensitivity (`0.5` floor, mm ranking, `maxTraversalExtent` coupling).
- Change: keep voxel dedup as the cheap first pass; add a second suppression keyed on segment-to-segment
  distance (all four endpoint↔opposite-segment distances < threshold), keeping the shorter/better-ranked link,
  and never merging across `(targetIsland, directionClass)` — the invariant from fixed finding 4. Threshold 0
  disables (mirrors UE's "0 deactivates filtering"). Index with a uniform grid on segment midpoints; the grid
  choice itself is speculative until the allocation/collision benchmarks the internal audit requests exist.

### R5. Axis-separated, resolution-relative density scales (PROVEN direction, exact formula SPECULATIVE)

- Sources: UE `SamplingSeparationFactor × cellSize`, `FilterDistanceThreshold` in world units with 0-off;
  Unity radius/voxel terms; internal finding 7 (single `maxTraversalExtent` scalar couples drop capability to
  lateral density; `0.5` absolute floor; mm ranking).
- Change: split every density voxel/radius into horizontal and vertical components derived from
  `maxHorizontalGap` and `max(verticalUp, verticalDown)` independently (plus optional explicit-meter
  overrides, already supported). Derive sampling separation from the navmesh cell size where known
  (pass `cs`/`ch` through `QueryTuning` or build context), falling back to today's ratios. This is the
  internal audit's step 5, now with external corroboration — both engines root sampling in raster resolution,
  IslandGraph roots it in capability maxima.

### R6. Multi-profile configs in one build + per-direction cost metadata (PROVEN pattern — adopt selectively)

- Sources: UE `NavLinkJumpConfigs[]` looped config-major; `DownDirectionAreaClass`/`UpDirectionAreaClass`
  with equal→bidirectional / different→two-directed / null→suppressed mapping.
- IslandGraph already matches the bidirectional-vs-directed representation more elegantly (one `Edge`, two
  flags). What is genuinely missing: (a) running Drop-like and Jump-like profiles in a single build instead of
  rebuilding per capability set; (b) per-direction area/flag metadata for query-time costing. (a) falls out of
  looping `discoverCandidates` per gap profile with profile-tagged dedup keys; (b) is two `uint8/uint16` fields
  on `Edge` plus serializer version bump. Both proven; neither urgent — schedule behind R1–R3.

### R7. Extremity emission for wide spans (PROVEN, niche — defer)

- Source: UE `DT_NAVLINK_CREATE_EXTREMITY_LINKS`.
- Single center corridors under-represent spans many agent-widths wide (all agents funnel through one point).
  Emitting span extremes (or center + extremes) for spans above a width threshold is proven in UE, but
  IslandGraph's local pruning would immediately collapse them without a width-aware exemption. Defer until R2
  span discovery exists; then gate on `spanWidth > extremityThreshold` (default off).

### R8. Traversal-behavior binding (PROVEN — small API addition)

- Sources: UE `LinkProxyClass`/`GeneratedNavLinksProxy` + `INavLinkCustomInterface`; Unity `OffMeshLink`
  component; IslandGraph already has `LinkRanker`, `PolygonFilter`, `OutboundIslandFilter`, `PathOptions`
  cost/filter/heuristic hooks.
- Gap: rankers/filters decide at build/query time, but there is no per-link runtime hook for execution
  (animation, jump physics, one-shot effects). Add optional `linkUserData`/`linkBehaviorId` on `Edge`
  (default 0 = none), populated from a `LinkBehaviorAssigner(link, graph) -> uint64` build callback and
  visible to `IslandGraphPathfinder` consumers. Proven pattern, tiny surface, no behavior change when unset.

### R9. Debug sampler + per-tile-style timing (PROVEN — extend existing stats, do not replace)

- Sources: UE `duDebugDrawNavLinkBuilder` (edges, trajectories, ground segments, filtered vs valid),
  `LinkGenerationDebugFlags` + selected-edge/config rebuild, per-tile link build time; Unity Bake
  troubleshooting section.
- IslandGraph's `BuildStats` (boundary sampling rows, candidate counters, direction validity, unreachable
  components) already exceeds both engines' telemetry. Add: rejected-by-trajectory counter, per-profile
  candidate counts, and an optional debug dump of sampled spans/trajectories for a selected island pair
  (the UE selected-edge workflow). This converts future tuning arguments from opinions to traces.

### R10. What explicitly NOT to do

- Do not adopt Unity's same-level Jump-Across gate or UE's down-biased single config as the only profile;
  IslandGraph's independent climb/drop limits + asymmetric one-way edges are the more general model (keep).
- Do not adopt UE's O(n²) overlap filter verbatim at global scale; keep voxel first-pass + indexed second pass.
- Do not move generation to tile-bake time; IslandGraph's post-build global view is what enables island
  topology, spanner pruning, and high-level routing — the engines' tile-local scope cannot do any of that.
- Do not add the deferred third-party stack (nanoflann/libmorton/Taskflow) as a precondition. R2's per-edge
  sampling does make future edge-parallel execution viable (UE's per-edge loop is embarrassingly parallel),
  but the internal audit's sequencing stands: correctness stages first, whole-build optimized baselines second,
  libraries only on measured residual cost.

## 6. Suggested implementation order (merges internal roadmap with external findings)

1. R1 validator hook + built-in parabola behind a flag (fixes the largest correctness gap; unblocks honest
   recall/precision measurement).
2. R2 edge-interior sampling + R3 takeoff/tolerance margins (closes internal finding 8; makes R1's validation
   meaningful along the whole edge).
3. R4 segment-distance suppression + R5 axis-separated scales (replaces the most unit-sensitive heuristics;
   re-run the internal audit's scale/translation/retessellation invariance checks here).
4. R9 telemetry additions (prove 1–3 with counters and sampler dumps before further tuning).
5. R6 multi-profile + per-direction metadata + serializer bump; R8 behavior id; R7 extremity emission last.
6. Only then: parallelize the per-edge sampling loop and re-evaluate the deferred library candidates against
   optimized baselines.

## 7. Validation checklist for the above

- Parabola/trajectory rejection cases: wall between platforms, low ceiling over gap, post in flight path —
  accepted today, must be rejected with R1 on.
- Long-edge endpoint crossings discoverable after tessellation changes (internal finding 8 repro).
- Knife-edge projections (takeoff/landing within `takeoffOffset`/`endpointTolerance` of non-standable space)
  rejected with R3 on, accepted with defaults off (backward compatibility).
- Near-parallel offset corridors: suppressed under R4 threshold, retained at 0; direction classes never merge
  (internal finding 4 invariant).
- Scale/translation/retessellation invariance suite passes after R5 (internal finding 7 repro).
- Serialized graphs from prior versions either migrate or are rejected with a clear version error; rebuild
  guidance from `README.md` ("Rebuild cached graphs after upgrading...") is repeated for the new version.

## 8. Sources consulted

- Unity: https://docs.unity.cn/Manual/nav-BuildingOffMeshLinksAutomatically.html
- Unreal docs: https://dev.epicgames.com/documentation/unreal-engine/automatic-navigation-link-generation?lang=en-US
- UE source: `Engine/Source/Runtime/Navmesh/Public/Detour/DetourNavLinkBuilder.h`,
  `Engine/Source/Runtime/Navmesh/Public/Detour/DetourNavLinkBuilderConfig.h`,
  `Engine/Source/Runtime/Navmesh/Private/Detour/DetourNavLinkBuilder.cpp`,
  `Engine/Source/Runtime/Navmesh/Private/Detour/DetourNavLinkBuilderConfig.cpp`,
  `Engine/Source/Runtime/Navmesh/Private/DebugUtils/DetourNavLinkDebugDraw.cpp`,
  `Engine/Source/Runtime/NavigationSystem/Public/NavMesh/LinkGenerationConfig.h`,
  `Engine/Source/Runtime/NavigationSystem/Private/NavMesh/LinkGenerationConfig.cpp`,
  `Engine/Source/Runtime/NavigationSystem/Private/NavMesh/RecastNavMeshGenerator.cpp`
  (`AddGeneratedLinks`, `BuildTileCacheLinks`, `ComputeConfigBorderSizes`, config propagation),
  `Engine/Source/Runtime/NavigationSystem/Public/NavMesh/RecastNavMesh.h`,
  `Engine/Source/Runtime/NavigationSystem/Private/NavMesh/RecastNavMesh.cpp`.
- Local: `include/detour_island_graph/*.h`, `src/*.cpp`/`*.h`, `tests/IslandGraphTests.cpp`, `temp/AUDIT.md`.
