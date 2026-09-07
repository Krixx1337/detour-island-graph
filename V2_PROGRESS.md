# V2 MVP implementation status

## Revised MVP scope, 2026-09-07

The revised [MVP plan](temp/todo/V2_MVP_PLAN.md) controls scope. Delivered sections
below describe the existing baseline, not completion of the expanded MVP. The
follow-up chat was consolidated into the plan and removed.

Pending additions: topology/sampling separation, island metrics, reasoned domain
selection, seeded validated frontier expansion, bounded production processing,
one working host validator, mass-policy migration, lazy native Detour transfers,
and domain-aware persistence and query results. This documentation revision does
not implement these features. Existing host remains on V1.

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
  adjacency offsets, and polygon-to-island lookup. Routing was delivered later
  in this baseline; see its section below.
- Input checks for finite geometry/settings, topology ownership, canonical order,
  directional consistency, and conflicting duplicate results.
- Cancellation and callback failure return no output, even after partial work.
  Candidate cap applies to raw input before exact deduplication. Exactly the cap
  is allowed; exceeding it fails the stage. Sampling below adds sample caps.
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
candidate discovery below establishes projected landing ownership.

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

## Routing delivered

Implemented in `src/v2/Routing.cpp`, declared in
`include/detour_island_graph/v2/Routing.h` as `findRoute` over
`CompiledGraph`:

- Portal-based search over compiled directed traversals using the graph's
  precomputed offsets. No mutable search state in the shared graph;
  caller-owned `RouteScratch` holds states and the heap, reusable across
  queries with identical results to scratch-less calls.
- `TransferCost` receives the island plus anchored endpoints (ad-hoc query
  positions carry polygon 0); `CrossingCost`/`CrossingFilter` receive the
  crossing, direction, and route context. Null costs mean Euclidean.
- Geometric A* runs only under fully default costs with an explicitly
  `estimatedCost` label. Any custom cost provider switches to Dijkstra; no
  custom heuristic callback exists. Non-finite or negative costs block that
  candidate rather than poisoning the search; callback exceptions fail the
  query without partial output.
- Search skips expansion when accumulated cost cannot improve the best completed
  route, but does not implement the planned best-remaining-bound early exit.
  Same-island queries return `SameIsland` with no search.
- Current `estimatedCost` follows default-cost/A* selection. A custom crossing
  cost clears it even when transfers remain Euclidean. Separate cost provenance
  remains required.
- No native Detour transfer integration or transfer cache is implemented. Lazy
  host transfers and a bounded per-query cache are now MVP requirements;
  cross-query caching remains deferred.
- Statuses cover success, same-island, no-path, invalid islands/input,
  budgets, cancellation, callback failure, and out-of-memory, with
  expanded/queued/peak-open telemetry.

## Serialization delivered

Implemented in `src/v2/Serialization.cpp`, declared in
`include/detour_island_graph/v2/Serialization.h` as `GraphSerializer`
(magic `"DIG2"`, format version 1, little-endian IEEE-754 bytes like V1):

- Stores island/polygon/crossing counts, full build identity, policy flags,
  discovery settings, compilation policy, sorted polygon ownership, and every
  compiled crossing with anchored endpoints plus per-direction eligibility,
  policy, and validation records. Polygon order is sorted for deterministic
  bytes; adjacency is never stored.
- Decode revalidates through `compileGraph` itself: counts against safety
  limits and an allocation budget, references, finite geometry, directional
  states, and canonical order are all rechecked, adjacency is rebuilt, and
  the blob is accepted only when every stored crossing survives compilation
  under the stored policy. V1 blobs fail on magic; wrong versions, truncated
  or out-of-range data, and budget or limit breaches fail without a graph.
- Identity, discovery settings, and policy round-trip exactly so hosts can
  compare cache identity before reuse. Unversioned custom semantics stay
  visible via the preserved `persistentReuseEligible` inputs.
- `CompiledGraph` now retains its provenance flags (`customPolygonPolicy`,
  `validatorSupplied`, `customOutboundPolicy`) as read-only accessors.
- Package version and V1 contracts are untouched until host migration; this
  format is V2-only.

## Verification

Previously recorded implementation results below were not rerun during the plan
revision and do not establish completion of the expanded MVP.

- Targeted Windows/MSVC Debug library test build passed.
- All 78 tests passed: 35 existing V1 tests, 11 V2 contract tests, 15 V2
  topology/sampling tests, 5 V2 discovery tests, 4 V2 pipeline tests,
  5 V2 routing tests, and 3 V2 serialization tests.
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

1. Capture fixtures and baseline on Queensdale plus an interior-heavy stacked
   workload. Inventory collision access, trusted seeds, movement rules, and costs.
2. Split topology from sampling; add metrics, domain reasons, seed coverage,
   exclusion propagation, and semantic identity.
3. Integrate host validation, seeded expansion, and bounded processing. Current
   `PipelineResult` retains all artifacts and nearby-polygon collection has no
   separate temporary-memory cap; both need production changes.
4. Add mass policy and lazy native transfers, fix routing gaps above, and extend
   the existing serializer for domain and metric contracts with a format bump.
5. Pass revised acceptance gates before host/settings/report migration and
   package version 2.0.0. Measure memory and query latency before deciding whether
   spans or regional routing must enter scope.
