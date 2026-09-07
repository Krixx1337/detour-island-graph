# V2 MVP implementation status

## Foundation delivered

Read all three inputs in `temp/todo`: `V2_MVP_PLAN.md`,
`EVOLUTION_RECOMMENDATIONS.md`, and `COMPARATIVE_AUDIT_UNITY_UNREAL.md`.
The MVP plan controls scope. Evolution recommendations correct the comparative
audit's proposed Detour flight validator and its bake/post-process dichotomy.

Implemented in `include/detour_island_graph/v2/Build.h` and `src/v2/Build.cpp`:

- Typed build identity, explicit discovery settings, polygon anchors, topology,
  validation requests, reusable crossing artifacts, and stage results.
- `validateCrossings` accepts already-discovered anchored candidates. It checks
  independent horizontal/climb/drop limits and outbound policy, then calls the
  validator separately for each eligible direction.
- One canonical geometry with AB/BA results. Exact duplicate anchors are merged;
  nearby geometry and different polygon anchors remain distinct. No approximate
  pruning. Equal limits never imply reverse validity.
- `compileGraph` recompiles an artifact under geometric-only or validated-only
  policy without running validation again. Invalid directions never traverse.
  Validated-only requires a supplied validator and excludes Unknown results.
- Immutable graph publication, contiguous outgoing adjacency, precomputed
  adjacency offsets, and polygon-to-island lookup. Routing portal-state layout
  is still pending.
- Input checks for finite geometry/settings, topology ownership, canonical order,
  directional consistency, and conflicting duplicate results.
- Cancellation and callback failure return no output, even after partial work.
  Candidate cap applies to raw input before exact deduplication. Exactly the cap
  is allowed; exceeding it fails the stage. Sample caps await sampling.
- Persistent-reuse eligibility requires versioned mesh/profile and any custom
  policies/validator/environment. This is an eligibility flag, not a cache-key
  implementation or serializer.

## Topology and sampling delivered

Implemented in `src/v2/Sampling.cpp`, with public artifacts and entry point in
`include/detour_island_graph/v2/Build.h`:

- `extractAndSample` reads a frozen final `dtNavMesh`, selects eligible native
  ground polygons, and creates deterministic dense island IDs. Off-mesh action
  polygons never join native topology, even when a custom polygon filter is
  supplied.
- Island and sample ownership follows tile coordinates, layer, and polygon
  index rather than tile allocation order. Polygon references remain tied to
  the input mesh snapshot.
- Native ground adjacency must be reciprocal. A malformed or one-way native
  relationship fails instead of merging polygons and falsely allowing reverse
  on-island travel.
- External portal coverage is unioned from all eligible linked neighbors.
  Only uncovered intervals remain boundaries, so partial portals and multiple
  linked spans no longer hide an entire polygon edge.
- Boundary intervals retain island, polygon, edge, normalized interval, and
  endpoint provenance. Final-navmesh extraction treats unresolved external
  seams as exposed because MVP assumes a complete snapshot and defers streaming
  semantics.
- Every exposed interval is sampled at both endpoints plus evenly spaced
  interiors. Three-dimensional edge spacing never exceeds the explicit
  `sampleSpacing`, apart from float representation limits. Sampling density is
  independent of island size and climb/drop reach.
- Exact coincident samples deduplicate only within the same island. Ownership is
  deterministic; coincident samples on different islands remain separate.
- `maxSamples` limits unique samples. Exceeding the cap returns
  `BudgetExceeded` with no artifact. Cancellation, malformed mesh data, callback
  failure, and float-resolution collapse also publish no artifact.
- Stage counters now expose ground polygons visited, eligible polygons, islands,
  boundary intervals, sample attempts, duplicate samples, and stored samples.

No v1 mass quota, representative reduction, voxel merging, pair suppression,
recovery pass, or pruning heuristic enters this stage.

V2 currently lives in a separate namespace while replacement proceeds. There are
no V1-to-V2 compatibility wrappers. Existing host still uses V1. Public package
version and cache formats remain unchanged until the complete replacement lands.

## Trust and ownership contracts

Directly supplied topology and candidates remain trusted producer inputs, not
serialized or hostile input. The compiler checks consistency against the
supplied polygon ownership table. `extractAndSample` now establishes topology,
polygon references, and boundary sample positions from the frozen Detour mesh;
future candidate projection must establish landing anchor ownership.

Callbacks must be deterministic and capture frozen state. Artifacts must not be
mutated concurrently with compilation. Validation results and their provenance
must remain paired; changing identity fields cannot revalidate geometry.

Teleport consumers can select geometric-only compilation. Unknown means no
confirmed traversal validation, not a collision-clear or ballistic jump claim.
Movement execution remains outside this library.

## Verification

- Targeted Windows/MSVC Debug library test build passed.
- All 61 tests passed: 35 existing V1 tests, 11 V2 contract tests, and 15 V2
  topology/sampling tests.
- Root architecture boundary checker and whitespace checks passed.

No host switch, full application build, Queensdale performance measurement, or
in-game execution check is part of this slice.

Relevant Unreal source was inspected at
`Engine/Source/Runtime/Navmesh/Private/Detour/DetourNavLinkBuilder.cpp`.
Unreal extracts pre-bake contours and samples ground against retained
heightfields. V2 uses portable final-Detour-mesh interval extraction instead;
no Unreal implementation or Unreal dependency was copied. Rich collision and
heightfield data remain caller-supplied validator concerns.

## Next slice

1. Discover projected anchored candidates with cancellable Detour queries and
   feed the validation/compiler stages. Enforce candidate cap while generating,
   not merely after allocating the candidate list. Avoid Detour's fixed-size
   polygon result overload so dense stacked geometry cannot truncate silently.
2. Add one end-to-end convenience build entry point once candidate discovery is
   complete. Preserve stage artifacts for testing, timing, and future reuse.
3. Benchmark the dense, exact-only pipeline on Queensdale before full host
   migration. Record sample/candidate/crossing counts, stage timings and peak
   memory. The current ordered-map implementation is a correctness baseline;
   profile its allocation and duplicate-key cost before optimizing it.
4. Implement V2 routing and caller scratch, new serialization/cache identity,
   then host/settings/report migration and package version 2.0.0. Benchmark query
   latency when the router is available.

Candidate discovery, end-to-end build, mass diagnostics, routing, serialization,
and host migration remain unfinished. V2 still cannot build a usable graph
directly from a navmesh without caller-supplied candidates.
