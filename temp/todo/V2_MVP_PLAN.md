# DetourIslandGraph V2 MVP

Revised 2026-09-09 after the autonomous acceptance audit of the extractor-owned
post-assembly traversal-build stage and portable library.
This document controls MVP scope. [V2_PROGRESS.md](../../V2_PROGRESS.md) records
delivered code and gaps. Requirements below are not claims of implementation.
The [acceptance matrix](../../docs/v2-acceptance.md) identifies named automated
evidence and deferred checks. The built-in minimum-area build policy was explicitly
deferred by the user after the audit and is no longer an MVP implementation blocker.

Current execution scope: extractor integration has resumed; benchmarking remains
deferred by user request. Final mesh assembly, teleport validation, pinned DIG
dependency, optional collision capture and snapshot coverage assessment are in
place. Optional Job-driven traversal analysis now runs before nav output, reports
diagnostics, and fails before export on traversal errors. DAT/OBJ coverage remains
incomplete by default. Extractor now supports opt-in single-file `navBundle` delivery,
bounded matching mesh/graph reload, and atomic publication after verification.
Existing nav-only output remains the default. Host migration remains deferred.
DAT and OBJ link-only rebuilds are now implemented through optional `inputBundlePath` jobs.
It reloads matching collision, verifies source/import provenance, preserves exact
baked nav bytes, and atomically replaces the graph without Recast. Both real
fixtures verify equivalence, including changed movement reach. DAT rebuild verifies
map payload, resolved identity, extraction flags, and separate bake/capture geometry
hashes. Synthetic map payloads exercise production parsing and rebuild without a DAT
archive; real archive/index acceptance remains deferred. Old bundles need a new full
bake to acquire rebuild provenance.
Extractor bundle operations now enforce a default 1 GiB aggregate logical working
budget, preflight native tile storage, avoid graph input copies, and release output
buffers before publication reload. Fixture reports expose accounted load/write
peaks. This excludes caller-owned data, JSON/codec internals and DIG compiler
overhead; it is not a process RAM cap. Strict allocation and whole-pipeline memory
limits remain future work. No host or DIG wire/API changes are required.
Host work is explicitly deferred. Extractor standalone-world tests now exercise
positive strict link generation on existing Pandora/Steelribs OBJ fixtures, byte
determinism, serialization and cross-island graph routing. Their test-only coverage
declarations describe the supplied OBJ world, not complete original-game collision.
Ordinary incomplete evidence still produces no executable links. See V2_PROGRESS.md
for measured results; native transfer routing is verified on fixtures. Live movement
calibration remains unverified.

Automated health checks now support fixture-first iteration. DIG exposes opt-in,
size-limited structural and directed-connectivity analysis. Extractor's development
runner checks every ordered included island pair against independent BFS, validates
route legs/costs, repeats batches and serialization, and writes JSON/text reports.
The test script compares repeated deterministic fields and checks source hashes.
Connectivity and timing statistics are descriptive; invariant failures, route
mismatches, lost fixture expectations, and exhausted budgets fail validation.
Production health reporting, host work, and in-game testing stay deferred. Current
runner requires the local DIG source override until its API is published.

Adversarial tests now check discovery against independent rectangle arithmetic,
document a coarse/fine sampling landing window, and exercise up to 1,024 sparse
islands and 256 overlapping layers. Collision scenarios assert destination-only
teleport semantics, including clear hollow interiors that legitimately pass this
model. Reports track scenario expectations and resource use. The built-in
minimum-area build policy and performance redesigns remain separate future work;
soft/strict route-area preferences are implemented.

## Direction

Keep V2's directional crossing model and immutable graph compiler. Expand MVP to
separate native topology, playable-domain selection, traversal validation, and
route preference. Dirty collision-derived navmeshes are a primary workload.

Primary deployment has raw collision and an owned Recast bake pipeline, but no
manual seeds or trusted landmarks. Build V2 after final navmesh assembly while a
matching frozen collision snapshot is available. Target collision-checked crossings
under the host's actual movement model. Keep final-navmesh-only geometric output as
an explicit fallback for callers without that evidence. Classifying every island
as playable or garbage is not required for MVP.

Exhaustive autonomous construction is the initial implementation and correctness
reference under configured sampling. Its suitability for production dirty maps
remains unproven until workload measurements exist. Seeded construction remains
supported when a caller has real trusted anchors, but MVP cannot depend on them.

An island is a native connectivity group. Size does not prove playability, safety,
clearance, or short travel between portals. Final navmesh alone cannot reliably
distinguish legitimate caves from unwanted hollow interiors.

Topology metrics may prioritize work and drive opt-in heuristic labels. They must
not silently turn the largest island into trusted ground or classify small islands
as garbage. A large sealed interior can outrank playable ground; tiny platforms and
thin vertical traversal surfaces can be legitimate. Autonomous cleanup from final
navmesh geometry is useful as an explicitly lossy policy, not an MVP guarantee.

Keep portable final-navmesh input. Allow frozen collision access or retained bake
data through the calling application. Generate traversals beside baking without
requiring Recast internals or retaining all heightfields. Remain C++17 with no new
third-party dependencies. Movement execution stays in the host.

Scheduling link generation after baking and storing links as native Detour off-mesh
connections are separate choices. Keep generated crossings in V2 for MVP. Reuse the
baked navmesh and collision acceleration structure when movement settings change;
rerun affected discovery and validation. Agent radius, height, or other ground-bake
settings may require rebaking. Native off-mesh storage is not inherently broken and
does not inherently require revoxelization when link settings change, but adopting
it is unnecessary for this MVP.

Audit existing bake filters before tuning them. Low-height and ledge filtering
address agent walkability; `minRegionArea` removes qualifying small regions in cell
units. These filters do not identify all interiors and can remove useful surfaces.
Use fixtures and recorded settings, with no claimed garbage-removal percentage.
Collision checks can reject blocked crossings into sealed geometry; they do not
prove that an open interior belongs to the intended playable world.

### Traversal-build placement and ownership

V2 runs as a dedicated stage in Gw2CollisionExtractor after ground baking and
assembly of every tile in the declared snapshot into a frozen final `dtNavMesh`.
Retain matching collision through validation, then export the navmesh and compiled
graph for the host. Do not generate jumps inside individual Recast tile builds.

DetourIslandGraph owns portable topology, sampling, discovery, validation contracts,
and graph compilation. The extractor owns collision lifetime, acceleration queries,
final mesh assembly, and execution of the build stages. The host supplies versioned
movement settings and policies, schedules worker jobs, and owns routing, movement
execution, cache acceptance, and immutable publication. Implement the validator
outside the portable library with semantics matching the host controller. Keep one
builder and one versioned movement definition across producer and consumer.

The original pre-implementation inspection of Gw2CollisionExtractor established
the following baseline. Final assembly and traversal validation have since shipped,
as recorded above:

- `src/Processors/NavMesh/RecastProcessor.cpp` builds Detour tile blobs and writes
  them with `tileRef = 0`; it does not assemble a final `dtNavMesh`. The consumer's
  `Assets/Loaders/RecastMeshLoader.cpp` assembles the mesh using `addTile` with
  automatically assigned references. Final native connectivity and polygon
  anchors must be established before V2 extraction.
- `BuildNavRegions` frees the solid heightfield after compact-heightfield
  construction. `BuildPolyMesh` frees compact data after detail-mesh construction.
  Retaining heightfields requires explicit lifetime and memory changes. Existing
  tile borders are sized for ground baking, not arbitrary cross-tile trajectories.
- Indexed collision triangles and `rcChunkyTriMesh` remain available during
  `Generate`. The chunk index offers horizontal broad-phase queries, not an
  agent-volume collision validator. Reuse is possible only with a suitable
  narrow-phase implementation and frozen lifetime.
- `src/Core/ExtractionPipeline.cpp` can filter underwater geometry, omit blockers,
  and continue after HAVK parse failure. Collision export is a separate debug-only
  mode; the production navmesh output does not hand collision to the host.
  Successful baking alone does not establish complete collision evidence.

Historical inspection of the obsolete GW2NavMeshBuilder and host loaders also
established the following. GW2NavMeshBuilder is not the current implementation:

- `GW2NavMeshBuilder.cpp` reads OBJ, bakes tile blobs, and writes `.nav` with zero
  tile references. It supplies neither final mesh assembly nor a jump validator.
- `CollisionMeshLoader.cpp` can load `.gw2mesh` and build a horizontal spatial grid.
  This is collision-loading support, not an existing swept-agent validator.
- `MapAssetService` selects Recast or collision data; both loaders clear their
  destination. Paired navmesh/collision snapshots need separate ownership and
  lifecycle work before host-side generation could use them together.
- `Gw2MeshFormat.h` uses map-wide 16-bit coordinate quantization and lacks collision
  revision, transform, and coverage metadata. Authoritative clearance from this
  format would require stronger identity and explicit quantization-error handling.

Prefer worker generation for MVP because collision is already resident during the
bake and routing needs the compiled graph. Avoid mandatory collision export,
decode, indexing, and map-sized collision storage in the host. Preserve obstacles
needed for execution checks independently of ground walkability policy; record
omissions and return Unknown where collision coverage is insufficient. Collision
availability alone does not establish coverage. The destination validator has
shipped, with incomplete ordinary DAT/OBJ evidence remaining conservative.

Keep the library C++17 without GW2 parsing, Recast heightfield dependencies, or host
controller logic. Extend the extractor's stateless job/output contract explicitly
for movement settings and compiled graph delivery. Update its
`docs/timeless-cli-architecture.md`, job schema, reporting, and consumer contracts
together. The host still owns UX and policy; no extractor-owned presets are needed.

Split worker build/assemble/validate/write boundaries. Export a matched navmesh and
graph with versioned identity and a checked polygon-reference contract. Current
zero tile refs and automatic loader assignment are not sufficient proof that
worker graph refs match host refs. Establish matching assembly/ref identity or
checked anchor remapping, with compatible Detour configuration, before publication.
Native off-mesh export and a separate traversal executable remain outside MVP.

Provide a link-only worker rebuild path using the existing compatible navmesh.
After the extractor exits, raw collision and its index are gone: movement changes
require matching collision re-extraction, or a future worker-readable collision
cache. Re-extraction must not force Recast rebaking. Reuse resident collision/index
within a job; do not promise cross-job reuse without persistence. A worker-owned
collision cache and host collision loading remain optional future work, justified
by repeated tuning costs or an actual runtime collision-query requirement.

Placement alone does not reduce candidate count or prove a speedup. Prioritize
bounded batches, spatial destination queries, cheap directional rejection, exact
deduplication, and collision broad-phase queries over swept trajectory bounds.
Reuse compatible navmesh and collision acceleration across movement rebuilds.
Consider a 3D collision index for stacked maps only when measurements justify it;
retaining all bake intermediates is not the default optimization.

## 1. Topology and domain contracts

Split `extractAndSample` into topology extraction and independently scheduled
boundary sampling. Retain a convenience wrapper. Proposed flow:

```text
Extractor: raw collision -> Recast tile bake -> assembly -> frozen final dtNavMesh
Matching resident collision -> execution-matched clearance queries
Dedicated extractor traversal-build stage using DetourIslandGraph
    -> frozen mesh, collision evidence, and host-supplied movement profile
    -> native topology and metrics
    -> domain policy and seed resolution
    -> selected-island sampling, discovery, directional validation
    -> frontier expansion when seeded mode is selected
    -> immutable graph with domain coverage and provenance
    -> export matched navmesh and compiled graph
Host: checked load/publication -> routing with host costs and native transfers
```

- Flood-fill eligible reciprocal native ground connectivity only. Generated
  crossings and off-mesh actions never merge islands.
- Record surface area, bounds, polygon count, and host-supplied trusted evidence.
  Define area calculation and units. Preserve raw metrics in compiled output;
  normalized mass is a versioned route policy, not topology truth.
- Distinguish explicit exclusion, unexplored geometry, and unreachability under
  the declared movement profile and completed search. Preserve reasons.
- Apply polygon exclusions consistently to topology, discovery, seed resolution,
  host transfers, and ordinary host Detour paths. Exclusions that split native
  connectivity require recomputation before publication.
- Whole-island policy may skip sampling without destroying raw ownership and
  diagnostic evidence. Define raw-to-compiled ID mapping if output is compacted.
- Keep anchors tied to a frozen mesh revision. IDs are snapshot-specific.
- Use navmesh units internally; host converts world-unit settings at its adapter.

Hard exclusions require explicit host policy or sufficiently strong evidence.
Tiny-area cutoffs may be optional heuristics with reasons and overrides. No default
polygon-count cutoff, percentage suppression, bounding-volume cutoff, sky-exposure
rule, or automatic sink deletion. Small platforms, caves, and one-way destinations
must remain representable. No garbage-removal percentage without measurements.

## 2. Autonomous exhaustive and optional seeded modes

Use bounded exhaustive mode as the initial implementation and exact-only reference.
It requires no manual seeds. Collision-backed validation applies to the primary
deployment, with explicit geometric fallback:

1. Extract every eligible native island and retain its metrics and ownership.
2. Apply only explicit caller exclusions by default. Process islands in a stable
   deterministic order. Area-prioritized scheduling is optional and must preserve
   completed output; it does not reduce total work.
3. Sample and discover crossings across the declared domain in bounded batches.
4. Apply directional eligibility and execution-matched collision validation. Compile
   ValidatedOnly when the required checks are implemented. Without sufficient
   evidence, retain Unknown and use only explicit GeometricOnly output.
5. Preserve every island's domain state and every accepted direction's provenance.

Defer the built-in minimum-area threshold by explicit user direction. Reconsider
only when measured workloads justify the optional loss of small-island routes;
any future implementation needs a stable exclusion reason and policy identity.
Preserve all eligible islands by default. Defer built-in polygon-count, vertical-strip, density,
and cumulative-area classifiers until fixtures demonstrate useful tradeoffs.
Custom domain policies remain available. No fixed garbage-removal rate or claim
that heuristic exclusion validates geometry.

Largest-island or top-area selection may produce processing roots and route priors.
An optional future auto-seeded subset build may deliberately omit components to
reduce work. Such roots are untrusted; unvisited islands remain Unexplored and
coverage is explicitly limited. Geometric expansion retains Unknown directions and
publishes only geometric output. It must not manufacture Valid results to use the
existing trusted seeded builder. This extension is not a required MVP task.

Automatic roots cannot guarantee exclusion of interiors: geometric links can cross
walls and expand into junk. Resolve any generated anchor on its actual polygon;
an island bounding-box center paired with an arbitrary polygon is not valid.

Keep explicit seeded mode for callers that later obtain trusted player-ground
anchors and a real traversal validator:

1. Resolve verified player-ground anchors or checked landmarks onto eligible
   polygons. Check vertical layer and projection tolerance. A POI coordinate alone
   is not trusted ground. Reject ambiguous or unresolved required seeds.
2. Activate seed islands and sample their boundaries at configured spacing.
3. Discover nearby destinations, including inactive eligible islands.
4. Validate each eligible direction. Activate a destination only through a Valid,
   policy-allowed outgoing traversal from an active island.
5. Process newly active islands until frontier exhaustion. Reverse-only acceptance
   does not expand forward reach. Preserve AB and BA results independently.

Trusted seeded expansion requires a supplied execution-matched validator. Unknown directions
do not activate islands in this mode. Keep this contract separate from any future
geometric subset mode; the latter does not establish a trusted seeded domain.

Unseeded components remain outside the declared build domain, not confirmed junk.
Distinguish out-of-domain query endpoints from no route inside the domain. Persist
seed identity, mode, exclusion policy, coverage, and completion state. Completion
means frontier exhausted under configured sampling, producer, and movement model,
not continuous-space or whole-map completeness.

Subset processing can reduce work by omitting components even with heuristic roots,
but neither savings nor useful coverage is guaranteed. Reliable seeds and validation
are needed to justify a trusted reachable domain. Initial topology extraction and
nearby-target queries still cost work. Huge active islands remain expensive.
Erroneous native links or extraction holes can admit junk; those need upstream
exclusions or geometry repair. Missing seeds or movement types can omit legitimate
areas. These limitations must be visible.

## 3. Sampling, discovery, and resources

- Treat supplied mesh snapshot as complete; unloaded-neighbor semantics deferred.
- Extract exposed edge intervals. Subtract the union of eligible linked external
  portal intervals, preserving partial boundaries and endpoint provenance.
- Sample endpoints and evenly spaced interiors with positive explicit
  `sampleSpacing`. Preserve spacing on every processed island. Existing 4-metre
  host setting is a comparison starting point, not a recall guarantee.
- Keep spacing independent of mass, deduplication, and movement reach.
- Deduplicate exact coincident samples within the same island deterministically.
  Preserve distinct anchored approaches in crossing artifacts.
- Query nearby polygons without silent fixed-buffer truncation. Apply independent
  horizontal/climb/drop limits; keep pairs eligible in either direction.
- Discover inter-island crossings only. Same-island actions remain deferred.
- Remove V1 mass quotas, representative reduction, pair suppression, recovery
  scans, approximate voxel merging, and local/global/spanner pruning.
- Merge exact duplicate crossing anchors only. Keep exact-only reference mode.

Production pipeline processes candidates in bounded batches and does not retain
every intermediate artifact by default. Full artifacts remain opt-in for analysis
and reuse. Exact deduplication must span batches without merging distinct anchors
or evidence. Preserve deterministic results across batch sizes.

Batching bounds intermediate storage, not total discovery work, the global exact
deduplication index, or final graph size. Budget those separately. A clean
`BudgetExceeded` result establishes controlled failure, not production scalability.
Do not declare exhaustive processing sufficient for dirty maps without measurements.

Specify sample, candidate, temporary-query, and memory/work budgets. Sample and
candidate caps alone do not bound topology or nearby-polygon allocations. Exactly
the cap is allowed; exceeding it returns `BudgetExceeded`, counters, and no
publishable graph. Never thin samples silently or publish an interrupted frontier
as complete. Cancellation and callback failures abort publication through all
stages. Failed rebuilds preserve the previous compatible immutable graph.

## 4. Traversal evidence and execution-matched validation

Raw collision is available in the extractor. Ship one working worker-side validator
matching actual host gap execution as part of integration acceptance. A callback contract alone is insufficient.
Current library-only work keeps this integration boundary testable; it does not
claim that the real validator has shipped. Missing evidence stays Unknown.

Prefer existing collision queries or an acceleration structure over the frozen raw
triangles. Inventory coordinate transforms, collision completeness, sidedness,
agent shape, endpoint support, and movement execution before choosing the backend.
Retained solid heightfields are an alternative if their resolution and spatial
coverage support the required clearance checks. Compact walkable data alone is not
equivalent to solid collision. Cross-tile trajectories need collision coverage over
the entire swept agent volume. Retaining all bake intermediates is not required.

One offset centerline ray cannot establish jump validity. Physical traversal needs
agent-volume clearance along the executed trajectory, with conservative treatment
of discretization, plus takeoff and landing checks. A ray-only implementation must
not promote an otherwise Unknown physical crossing to Valid. For callers without
collision, endpoint geometry remains a rejection filter and survivors stay Unknown.

Each canonical crossing stores geometry once with independent AB/BA records for
geometric eligibility, outbound permission, Valid/Invalid/Unknown, and reason.
Validate every eligible permitted direction independently. Equal limits never
prove reverse validity. Keep one movement profile per build.

- Physical jumps require supported endpoints, agent-volume clearance, and feasible
  trajectory under the controller's model. Straight endpoint rays or Detour surface
  raycasts cannot certify airborne traversal.
- Teleport-style gaps require actual host destination and execution rules.
  Ballistic feasibility is not their definition. Playable-domain policy remains
  separate from mechanical ability to reach an interior.
- Retain landing adjustment only with compatible validation: validate adjusted
  anchors or a documented admissible region, and recheck execution changes.
- Record movement profile, validator semantics, environment revision, and units.
  Policy recompilation does not revalidate another agent or environment.
- Validated-only compilation accepts Valid directions and requires a validator with
  evidence beyond final navmesh surface geometry.
  Explicit geometric-only compilation accepts eligible Valid/Unknown directions,
  never Invalid, and makes no collision-clear or playable-world claim.
- Validate before any future approximate elimination. Omit crossings without
  usable compiled directions. Execution and runtime rechecks remain host-owned.

Unreal reference: separate ground sampling and solid-heightfield trajectory checks
in `DetourNavLinkBuilder.cpp`, especially `sampleGroundSegment`,
`isTrajectoryClear`, and `checkHeightfieldCollision`. Borrow the separation, not
its complete implementation or assumed movement semantics. Recast small-region
filtering is supporting cleanup, not interior classification; inspected Unreal
`RecastRegion.cpp` also preserves border-connected regions.

Do not adopt the chat's unconditional native-link overflow, 50-metre node exhaustion,
or sub-millisecond routing claims. Limits depend on the Detour fork, configuration,
and workload. The inspected Unreal fork has a 16-bit static `maxLinkCount` and also
dynamic link storage; that is not a universal argument against native off-mesh links.

## 5. Mass preference and route quality

Keep portal routing and integrate actual native transfers in the host. Existing
host already applies intermediate-island road bias and human-effort cost through
`DetourIslandGraphAdapter.cpp`; preserve intentional semantics during migration.

Prioritize area-based route preference and native transfer costs over additional
cleanup classifiers. These address route selection while retaining small stepping
stones. They do not certify that a selected crossing avoids a wall. Library work
includes a reusable Detour transfer provider and bounded per-query cache; host work
supplies execution-specific costs, units, and adapter integration.

- Prefer soft nonnegative penalties for entering small intermediate islands.
  Query-time soft and strict controls are implemented; see
  [area preference contracts](../../docs/v2-area-preferences.md).
  Keep strict minimum-intermediate-mass policy explicitly selectable; it may
  intentionally return no path. Preserve intended destination exemptions.
- Prefer area over polygon count. Use stable world-unit thresholds or versioned
  normalization. Unrelated junk must not silently redefine preference.
- Keep penalties and movement costs in the same units. Mass is a preference proxy,
  not proof of local width, clearance, or safety.
- Evaluate lazy host Detour transfers during search with a bounded per-query
  cache. Constrain transfers to matching native island and polygon policy; exclude
  generated-action shortcuts and reject partial/truncated path results.
  The optional library provider is now implemented; see
  [native transfer contracts](../../docs/v2-native-transfers.md). Host adapter
  integration and execution calibration remain deferred.
- Resolve query start/end polygon anchors explicitly. Current polygon-zero query
  endpoints need checked host resolution before native transfer evaluation.
- Refining only the winning Euclidean route does not establish correct ranking.
  Native costs improve ranking but do not prove continuous-space optimality.
- Default library Euclidean transfers remain available and labeled estimated.
  Track transfer/crossing cost provenance independently of search algorithm;
  a custom mass callback must not make Euclidean transfers appear measured.
- Built-in Euclidean costs use geometric A*. Custom costs use Dijkstra in MVP.
  Accept finite nonnegative costs; represent blocked transfers explicitly.
- Implement best-remaining-bound early termination, accounting for stale queue
  entries, and verify default and custom costs.
- Reuse immutable adjacency, precomputed offsets, and caller-owned scratch.
  Same-island queries retain `SameIsland`; host handles native path with matching
  domain policy. No same-island shortcut search or cross-query transfer cache.

## 6. Persistence and host migration

- Extend existing V2 serializer for metrics, domain coverage, reasons, and policy
  provenance. Bump native format and host cache versions; reject older blobs.
- Identity covers mesh, units, polygon/exclusion policy, seeds/build mode,
  discovery settings, movement profile, validator/environment, and compile policy.
  Cost-only route preference changes should not force geometry rebuilds.
- Pair collision revision and transform with the baked mesh revision. Changed
  collision or movement semantics invalidate validation. Freeze their lifetimes
  through the build and reject mixed revisions before publication. Reuse topology
  when compatible; larger discovery reach requires discovering new candidates.
- Unversioned custom build semantics disable persistent reuse. Validate decoded
  counts, geometry, refs, mappings, direction states, and coverage consistency.
- Preserve protected-cache wrapping, atomic replacement, revision checks, and
  immutable publication. Deliver navmesh and compiled graph as a matched revision;
  reject mixed, incomplete, or incompatible outputs and preserve the previous
  compatible publication on failure. Among V2 artifacts, persist compiled graphs
  only in MVP; intermediate discovery/validation artifacts remain in memory.
- Version the worker graph delivery contract and verify polygon references after
  host loading. Test tile order, reference assignment, Detour configuration, and
  stale/mismatched graph rejection. Add a link-only job path that loads the existing
  navmesh and re-extracts matching collision without rebaking ground geometry.
- Migrate adapter, settings, callbacks, reports, UI, and cache together. Remove
  retired heuristic controls; expose soft versus strict policy and seeded-domain
  coverage. Keep existing maintenance lane and host movement ownership.
- Set package version to 2.0.0 at replacement. No V1 compatibility wrappers or old
  configuration-key migration required. Host stays on V1 until acceptance passes.

## 7. Diagnostics and acceptance

Report raw/eligible/excluded/active/unexplored islands and polygons, exclusion
reasons, whether policy decisions are explicit or heuristic, seed source and trust,
frontier completion, spacing, queries/projections, exact duplicates, directional
outcomes, graph size, stage timing, and peak memory.
Separate exclusion from validated unreachability and build failure. Expensive
nearest-representative diagnostics remain opt-in.

Fixtures and focused checks must cover:

- Valid tiny platforms, large sealed interiors, legitimate caves, stacked layers,
  largest-island misclassification, ambiguous seeds, one-way destinations, and
  erroneous native connectivity.
- Seed chains through small islands, reverse-only links, Unknown during expansion,
  missing seeds, out-of-domain queries, and exclusion-induced topology splits.
- Clear/blocked host actions, thin obstacles, endpoint support, adjusted landings,
  asymmetric validity, and changed validation identity.
- Centerline-clear but capsule-blocked gaps, overhead trajectory obstacles,
  cross-tile collision coverage, mesh/collision transform mismatch, and missing
  collision regions returning Unknown. Compare bake-filter settings on valid tiny
  platforms, caves, and unwanted interiors before enabling more aggressive cleanup.
- Long-edge opportunities, partial portals, spacing on large islands, exact
  deduplication across batches, and alternative anchored approaches.
- Soft preference retaining necessary stepping stones, explicit strict rejection,
  long native detours changing ranking, partial/blocked transfers, cost provenance,
  search termination, and consistent cost units.
- Budgets/cancellation without publication, corrupt/old caches, changed domain
  identity, and deterministic batch/tile ordering.
- Scale, translation, and equivalent retessellation using geometric tolerances,
  not identical sample IDs or polygon-count-based semantics.

Benchmark Queensdale and an interior-heavy stacked map or extracted fixture before
full host migration. Hold capabilities fixed; compare default exhaustive output,
opt-in heuristic policies, and trusted seeded builds where real seeds exist. Measure
false links, lost valid exits, labeled-domain coverage, actual route cost/execution
outcomes, build time, memory, and query latency distributions. Dense sampling is a
reference, not continuous-space ground truth. High connectivity alone is not
acceptance.

Choose performance thresholds from host requirements and measured baselines. No
arbitrary one-second target or 100% connectivity requirement. Run relevant library
tests, host dependency/targeted compilation checks, then real routes, rebuild and
cancellation, and stale-cache checks. Record unavailable in-game/full application
checks explicitly; do not mark them passed.

## 8. Remaining implementation order and exclusions

Delivered: bounded exhaustive production, optional seeded builds, metrics/domain
serialization, graph health, soft/strict route-area preferences, native transfers,
Extractor assembly and destination validation, matched bundle delivery, OBJ/DAT
link-only rebuilds, and logical bundle memory limits. The acceptance matrix records
their automated evidence and the limits of those claims.

No known required feature implementation remains in the currently accepted
fixture-tested DIG/Extractor scope. Remaining work is maintenance and the deferred
validation/integration below, not a claim of full V2 rollout readiness.

1. Keep the autonomous acceptance command passing as implementation evolves.
   Close concrete contract failures with focused regression cases, preserving
   the existing real-fixture oracles and optional seeded coverage.
2. When evidence work resumes, verify actual DAT archive/index integration and
   production coverage, then calibrate movement semantics. Ordinary DAT/OBJ output
   must remain conservative until coverage is independently established.
3. Benchmark later per current user direction. Then resolve measured blockers and
   complete checked host loading/reference identity, policy propagation, movement
   execution, diagnostics/UI, cache replacement, and package version. Host and
   dependency publication remain deferred; fixture acceptance does not switch them.

Deferred: built-in minimum-area build-domain policy, universal collision engine or physical jump solver, authoritative
automatic interior classifier or sink pruning, built-in shape/statistical
classifiers, auto-seeded geometric subset builds, multiple movement profiles per
artifact, spans and approximate pruning, regional routing and same-island shortcuts,
incremental tile builds, intermediate V2 artifact disk caches, worker collision
caches, paired host collision loading, cross-query transfer caches, traversal worker
parallelization, native off-mesh export, and new dependencies.

Reconsider spans if measured candidate volume defeats bounded exact-only processing.
Reconsider regional routing if portal expansion/native transfers miss latency
requirements. Preserve endpoint provenance now; require measured benefit and
explicit loss/validity guarantees before either redesign enters scope.
Benchmarking remains deferred, so production scalability remains an open acceptance
question. Do not pivot architecture again solely on unmeasured cleanup claims.
