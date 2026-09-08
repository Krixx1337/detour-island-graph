# V2 MVP implementation status

## Revised MVP scope, 2026-09-07

The revised [MVP plan](temp/todo/V2_MVP_PLAN.md) controls scope. Delivered sections
below describe the existing baseline, not completion of the expanded MVP. The
follow-up chat was consolidated into the plan and removed.

Pending additions: one working execution-matched worker validator, mass-policy
migration, lazy native Detour transfers, and native transfer integration. Existing host remains on V1.

## Bounded production foundation, 2026-09-08

Delivered first library phase for the extractor-owned build architecture. No
extractor or host integration is included in this phase.

- Added `buildGraphBounded` and `buildSeededGraphBounded`. Both require explicit
  nonzero allocation/work/count limits, sample/candidate caps, and batch sizes.
  They return a completed immutable graph, counters, timings, and budget
  diagnostics; intermediate artifacts are not returned. Existing artifact APIs
  remain the unbounded analysis/correctness reference.
- Extract topology once and reuse polygon ownership plus per-island interval
  ranges. Stream samples/candidates in bounded batches. Exact sample deduplication
  stays per island; canonical crossing evidence spans all batches, including
  rejected directions. Seeded expansion uses the same production implementation
  and only Valid outgoing directions activate new islands.
- Added allocation-aware `BuildVector` and `BuildUnorderedMap` storage aliases.
  Artifact fields and graph accessors use these aliases; callers naming concrete
  `std::vector` types must use the aliases, iterators, or `auto`. Library-owned
  allocation requests include nested topology, query collection, deduplication,
  compilation scratch, and final graph storage. Allocation accounts survive graph
  return; moving storage preserves ownership and copies outside builds are unbounded.
- Budget accounting checks before allocation/growth and counts old and new storage
  during reallocation. Exactly the cap is allowed. Overflow and exhausted limits
  return `BudgetExceeded`; system allocation failure remains `OutOfMemory`.
  Diagnostics include exhausted resource, limit/attempted usage, peak requested
  bytes, work units, nearby refs, batch peaks, and unique crossing count.
- Byte guarantee excludes caller inputs, callback captures/allocations and
  `std::function` bookkeeping, allocator bookkeeping, and Detour internals. One
  Detour query uses a fixed 256-node initialization. This is not an RSS or elapsed
  time guarantee. Work counts explicit traversal/processing operations, not sorting,
  container bookkeeping, or Detour's internal traversal.
- Nearby collectors stop retaining results immediately on failure, then propagate
  failure after the Detour query returns. No truncated query can publish success.
  Cancellation combines input, validation, and compilation sources throughout.
  Callback exceptions remain `CallbackFailed`, including callback `bad_alloc`.
  Callback reentry does not inherit the outer build's resource account.
- Production limits/batch sizes are execution controls, not new graph identity.
  Native serializer layout remains format 3; package version remains unchanged.
  Persisted discovery sample/candidate caps retain their existing semantics.
- Verification: MSVC x64 Debug build and full library suite passed, 106 tests and
  3261 assertions. Fixtures cover invalid-input precedence, reference/batch equivalence,
  directional/domain policies, seeded chains/reverse-only/Unknown, every resource
  cap, rejected duplicates, dense stacked queries, partial portals/tile order,
  every exhaustive cancellation checkpoint, callback reentry, and allocation
  growth/lifetime/overflow.

Benchmarking remains deferred. Controlled completion/failure is implemented;
production scalability, actual collision validation, graph transport/reference
identity, link-only worker rebuilds, and host migration remain acceptance work.

## Library continuation, 2026-09-07

Benchmarking is deferred by request. Current work is library-only.

- Added `extractTopology`, returning native ownership, exposed intervals, and
  per-island polygon count, bounds, and coarse 3D triangle-fan surface area in
  squared navmesh units. Metrics do not classify playability or use detail relief.
- Added `sampleBoundaries` with optional explicit island selection. Unset selects
  all Included islands; empty selects none. Order and duplicates do not affect samples.
  Full eligible target ownership remains available to candidate discovery.
- Retained `extractAndSample` as the exhaustive convenience wrapper. Existing
  pipeline uses it. Extraction can now be reused without rerunning polygon filters.
- Verified selection, inactive-target discovery, wrapper equivalence, metrics on
  slopes, polygon exclusions, caps, cancellation, and callback failures.
- MSVC Debug library/test build and all 93 tests pass. Benchmarking and host checks
  were not run.

- Added `BuildInput::islandPolicy`, called once per native island with metrics.
  Decisions retain Included, Excluded, or Unexplored state and host reason. No
  default size cutoff exists. Polygon exclusions still require `polygonFilter`
  before native connectivity extraction. Whole-island decisions retain raw IDs.
- Sampling processes Included islands only. Discovery skips Excluded targets but
  retains Unexplored targets for future frontier validation. External excluded
  candidates never call the validator. Compilation keeps crossings only between
  Included islands and preserves full ownership, metrics, and domain records.
- Routing returns `OutOfDomain` before `SameIsland` for uncovered endpoints.
  Extraction and compilation count Included, Excluded, and Unexplored islands.
- Native format 3 persists metrics, domain states/reasons, domain-policy identity,
  and seed coverage. Formats 1 and 2 are rejected. Unversioned custom domain policy disables
  persistent reuse. Decode validates metric counts/geometry, domain flags/states,
  and compiled crossing coverage; new metadata allocations consume decode budget.
- Added end-to-end domain/persistence checks and malformed producer, corrupt
  metadata, callback failure, and cancellation cases.

## Seeded library pipeline delivered

- `buildSeededGraph` requires explicit polygon-anchored seeds and a validator.
  Every required seed must match eligible native ownership, project over its
  stated polygon, and stay within explicit 3D projection tolerance. The library
  does not guess nearest layers or establish host trust in a landmark.
- Non-excluded islands start Unexplored. Checked seed islands become Included.
  Only geometrically eligible, policy-allowed Valid outgoing crossings activate
  new islands. Reverse-only and Unknown results do not expand forward reach.
- Sampling/discovery run per active island. Exact canonical crossing keys span
  batches, avoiding repeated validation of duplicate anchors. Sample/candidate
  budgets apply cumulatively; each batch receives the remaining allowance.
  Exhaustion, callback failure, and cancellation return no graph.
- Complete frontier exhaustion publishes only ValidatedOnly output. Compiler
  checks canonical seeds, completed coverage, and reachability of every Included
  island from those seeds through compiled validated directions.
- Coverage stores seeded mode, completion, projected seeds, and host seed identity.
  Unversioned seed semantics disable persistent reuse. Host seed identity must
  version trust/projection semantics, including tolerance. Format 3 round-trips
  coverage and rejects incomplete or inconsistent data.
- Tests cover forward/reverse chains, Unknown, bad ownership/layers, excluded
  seeds, outbound policy, duplicate seeds, exact cumulative caps, callbacks,
  cancellation, and persisted coverage.

Unexplored never means unreachable. Completion is relative to configured sampling
and validator, not continuous-space completeness. Boundary extraction still visits
the full eligible mesh; each selected sampling pass scans intervals and copies
ownership. Nearby-query allocation, topology memory, and retained exact evidence
still need explicit production bounds. Host native paths must enforce the same
domain policy during migration.

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
- Search discards stale entries, then checks the best remaining bound before
  charging expansion budget. A proven route at exactly the cap succeeds.
  Non-improving successors are not queued. Improved states can reopen.
  Same-island queries return `SameIsland` with no search.
- Transfer and crossing estimate flags are independent of A*/Dijkstra selection.
  Custom providers default to estimated; callers may explicitly declare each
  component non-estimated. Missing providers always retain the Euclidean estimate
  flag. `estimatedCost` is true when either component is estimated, and
  `usedAStar` reports search ordering separately.
- Cancellation is checked on entry, after cost evaluation, and before publishing
  query output. Regression tests cover exact expansion caps, stale entries,
  competing arrivals with expensive final transfers, mixed cost provenance, and
  cancellation from the final transfer callback.
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

## Extractor standalone real-fixture validation, 2026-09-08

Extractor tests now build owned collision snapshots from the existing Pandora and
Steelribs OBJ sources and verify unchanged matching nav bytes. Test-only trusted
domains define the supplied triangles as the entire standalone world. Production
DAT/OBJ coverage remains incomplete.

Strict builds accept 91 Pandora directions and 365 Steelribs directions. Batch sizes
1 and 64 produce identical graph bytes, which also survive serialization round-trip.
A fixed Steelribs crossing has independently inspected source support, valid polygon
references, and a successful cross-island graph route before and after decode.
Removing the declaration yields no executable links and NoPath for those endpoints.
Extractor's three CTest suites pass in Debug and Release. This tests graph routing
with estimated transfer costs, not native Detour paths or live game execution.

## Extractor Job-driven analysis, 2026-09-08

Optional schema-2 Job traversal settings now run capture, snapshot construction,
and bounded strict discovery after assembly and before nav writing. All movement
settings and budgets are explicit; nested configuration is strict. Default DAT/OBJ
coverage stays incomplete. Zero valid directions is successful analysis, and the
in-memory graph is discarded after diagnostics. No graph output or reuse is enabled.

NDJSON retains existing event names and adds traversal progress and diagnostics.
Traversal failures use existing navmesh_generation_failed and stop before export.
CLI fixture checks cover enabled/disabled Jobs, unchanged Debug nav bytes, protected
Release runs, invalid configuration, and library/query budget failures preserving
existing files. Host code remains untouched and deferred.

## Next slice

Automated fixture health is now available before host work. The optional portable
`analyzeGraphHealth` API checks graph invariants and reports directed components,
distinct neighbors, isolation, and polygon/area-weighted reference reachability.
Analysis has explicit size limits and cancellation; it does not change serialized
graphs or run implicitly during builds. A completed analysis can contain issues;
callers must inspect `healthy()` as well as the result status.

Extractor's opt-in fixture runner shares the existing OBJ bake/collision helpers,
checks every ordered included island pair against independent BFS, validates route
legs and estimated costs, and repeats health/routing checks across batches and
serialization. JSON/text artifacts record identities, settings, coverage, validation
reasons, resource usage, and route outcomes. The development script repeats reports
and compares all fields except timings. New target uses the local DIG override;
the published dependency pin and production CLI contracts remain unchanged.

Standalone Pandora has 91 directions but only 4 successful ordered cross-island
pairs; Steelribs has 365 directions and 22 successful pairs. These are descriptive
measurements, not completeness targets. Incomplete evidence still produces zero
strict traversals. Host, in-game validation, and native transfer routing remain
deferred. See Extractor `docs/fixture-health.md` for commands and report semantics.

Validation: all 116 DIG tests passed in MSVC Debug and Release. All five Extractor
CTest suites passed in both configurations. Repeated reports matched outside
timings, source SHA-256 checks passed, and missing-fixture/output-write failures
returned nonzero. No host or in-game checks were required for this slice.

Extractor integration has resumed. Gw2CollisionExtractor now owns final navmesh
assembly, the execution-matched teleport destination validator, a pinned DIG
dependency, optional unfiltered collision capture, and owned collision snapshots.
Coverage assessment requires an explicit trusted source-domain declaration and
rejects partial parsing or capture defects. Ordinary DAT/OBJ evidence remains
incomplete; synthetic complete scenes exercise positive strict validation.

1. Establish real source coverage and calibrated movement inputs, then add matched
   graph delivery. Current CLI executes traversal analysis but does not export a
   graph; live-game completeness is unproven. Host work remains deferred.
2. Add soft/strict area preference and lazy native Detour transfers with checked
   query anchors and bounded per-query transfer cache.
3. Add the narrow opt-in minimum-area policy, remaining diagnostics and dirty-map
   fixtures, and update persistence when those contracts change.
4. Add checked host loading and link-only rebuilds using bounded production APIs.
5. Benchmark before host migration and package version 2.0.0. Measure candidate
   memory and routing latency before adopting spans or regional routing.
