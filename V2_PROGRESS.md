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

## Candidate discovery delivered

Implemented in `src/v2/Discovery.cpp`, declared in
`include/detour_island_graph/v2/Build.h` as
`discoverCandidates(const SamplingArtifact&, const dtNavMesh&,
const DiscoveryConfig&, const Cancel&)`:

- Uses only the collector `queryPolygons` overload with a default accept-all
  filter; the fixed-size overload is never used, so dense stacked geometry
  cannot truncate silently. Targets are resolved through the sampling topology
  index, so filtered, unknown, off-mesh, and same-island polygons never emit.
- Landing anchors carry the projected polygon, its island, and the closest-point
  position with finite checks and `-0` normalization. A pair is kept when at
  least one direction passes independent horizontal/climb/drop limits, so
  reverse-valid asymmetric pairs survive even though the reverse sample need
  not reproduce identical geometry.
- Raw output with no exact deduplication; `validateCrossings` remains the
  canonical exact-duplicate stage. Nearby refs are sorted and uniqued per
  sample for deterministic order and to avoid redundant projections.
- `maxCandidates` is enforced while generating; exceeding it returns
  `BudgetExceeded` with no output. Exactly the cap is allowed. Cancellation is
  checked per sample, per nearby polygon, and around queries; query-init OOM
  maps to `OutOfMemory`, `queryPolygons` failure and malformed anchors to
  `InvalidInput`, single `closestPointOnPoly` failures to a
  `projectionFailures` counter with skip, and callback exceptions to
  `CallbackFailed`.
- Stage counters add `discoveryQueries`, `nearbyPolygons`, `projections`,
  `projectionFailures`, and `candidatesVisited` (emitted raw candidates).

## End-to-end build delivered

Implemented in `src/v2/Pipeline.cpp`, declared in
`include/detour_island_graph/v2/Build.h` as `buildGraph(const BuildInput&,
const DiscoveryConfig&, const ValidationOptions&, const CompileOptions&)`:

- Runs extract, discover, validate, and compile with one consistent
  `DiscoveryConfig`. Sampling and discovery use `input.canceled`; validation
  and compilation use their own options' callbacks. No new validation logic;
  pure stage plumbing.
- Each stage's full `StageResult` is preserved in `PipelineResult` for testing,
  timing, and reuse, including recompiling a preserved `CrossingArtifact`
  under a different policy without revalidation. Stages after the first
  failure do not run and keep their default result; per-stage statuses are
  checked in order.
- Wall-clock `StageTimings` cover each attempted stage plus the total,
  providing the per-stage timing basis for the Queensdale benchmark.
- First failure status becomes the pipeline status; no partial graph is
  published. `bad_alloc` during plumbing maps to `OutOfMemory`.

## Verification

- Targeted Windows/MSVC Debug library test build passed.
- All 70 tests passed: 35 existing V1 tests, 11 V2 contract tests, 15 V2
  topology/sampling tests, 5 V2 discovery tests, and 4 V2 pipeline tests.
- Whitespace checks passed (no tabs or trailing whitespace in touched files).

No host switch, full application build, Queensdale performance measurement, or
in-game execution check is part of this slice.

Relevant Unreal source was inspected at
`Engine/Source/Runtime/Navmesh/Private/Detour/DetourNavLinkBuilder.cpp`.
Unreal extracts pre-bake contours and samples ground against retained
heightfields. V2 uses portable final-Detour-mesh interval extraction instead;
no Unreal implementation or Unreal dependency was copied. Rich collision and
heightfield data remain caller-supplied validator concerns.

## Next slice

1. Benchmark the dense, exact-only pipeline on Queensdale before full host
   migration. Record sample/candidate/crossing counts, stage timings and peak
   memory. The current ordered-map implementation is a correctness baseline;
   profile its allocation and duplicate-key cost before optimizing it.
2. Implement V2 routing and caller scratch, new serialization/cache identity,
   then host/settings/report migration and package version 2.0.0. Benchmark query
   latency when the router is available.

Mass diagnostics, routing, serialization,
and host migration remain unfinished.
