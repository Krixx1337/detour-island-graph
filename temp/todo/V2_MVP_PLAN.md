# DetourIslandGraph V2 MVP

Revised 2026-09-08 after dirty-map, routing, and raw-navmesh-only reassessment.
This document controls MVP scope. [V2_PROGRESS.md](../../V2_PROGRESS.md) records
delivered code and gaps. Requirements below are not claims of implementation.

Current execution scope: library implementation first; benchmarking is deferred
by user request. Host migration and its acceptance gates remain future work.

## Direction

Keep V2's directional crossing model and immutable graph compiler. Expand MVP to
separate native topology, playable-domain selection, traversal validation, and
route preference. Dirty collision-derived navmeshes are a primary workload.

Primary MVP input is a raw dirty `dtNavMesh` with no manual seeds, collision mesh,
editor metadata, or trusted landmarks. Deliver geometric routing that tolerates
clutter and selects useful routes under explicit movement assumptions. Classifying
every island as playable or garbage is not required for MVP.

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
data through the host. Generate traversals adjacent to baking when useful without
requiring Recast internals or retaining all heightfields. Remain C++17 with no new
third-party dependencies. Movement execution stays in the host.

## 1. Topology and domain contracts

Split `extractAndSample` into topology extraction and independently scheduled
boundary sampling. Retain a convenience wrapper. Proposed flow:

```text
Frozen mesh and host evidence
    -> native topology and metrics
    -> domain policy and seed resolution
    -> selected-island sampling, discovery, directional validation
    -> frontier expansion when seeded mode is selected
    -> immutable graph with domain coverage and provenance
    -> routing with host costs and native transfers
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

Use bounded exhaustive mode as the initial raw-navmesh-only implementation and
exact-only reference. It requires no external trust data:

1. Extract every eligible native island and retain its metrics and ownership.
2. Apply only explicit caller exclusions by default. Process islands in a stable
   deterministic order. Area-prioritized scheduling is optional and must preserve
   completed output; it does not reduce total work.
3. Sample and discover crossings across the declared domain in bounded batches.
4. Apply available directional checks. Without collision evidence, publish results
   as geometric candidates, never as collision-clear or playable-domain truth.
5. Preserve every island's domain state and every accepted direction's provenance.

Keep built-in heuristic scope narrow: an opt-in minimum-area threshold through the
domain policy, with a stable exclusion reason and policy identity. Preserve all
eligible islands by default. Defer built-in polygon-count, vertical-strip, density,
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

Trusted seeded expansion requires a supplied host validator. Unknown directions
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

## 4. Traversal evidence and host validation

Raw-navmesh-only MVP has no collision source and cannot ship a truthful physical
clearance validator. Its built-in geometric checks may reject impossible endpoint
height, distance, direction, or slope cases, but all survivors remain Unknown.
Detour polygons describe walkable surfaces, not intervening solids. Sampling a
parabolic arc against those polygons cannot prove that the arc avoids a wall.

When host collision or retained bake triangles become available, ship one working
host validator matching actual gap execution. A callback contract alone is
insufficient for `ValidatedOnly` output. Missing evidence stays Unknown, not Valid.

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
  Keep strict minimum-intermediate-mass policy explicitly selectable; it may
  intentionally return no path. Preserve intended destination exemptions.
- Prefer area over polygon count. Use stable world-unit thresholds or versioned
  normalization. Unrelated junk must not silently redefine preference.
- Keep penalties and movement costs in the same units. Mass is a preference proxy,
  not proof of local width, clearance, or safety.
- Evaluate lazy host Detour transfers during search with a bounded per-query
  cache. Constrain transfers to matching native island and polygon policy; exclude
  generated-action shortcuts and reject partial/truncated path results.
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
- Unversioned custom build semantics disable persistent reuse. Validate decoded
  counts, geometry, refs, mappings, direction states, and coverage consistency.
- Preserve protected-cache wrapping, atomic replacement, revision checks, and
  immutable publication. Persist compiled graphs only in MVP.
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

1. Finish resource bounds and bounded exhaustive batching for raw `dtNavMesh`:
   topology, sampling, nearby-query storage, retained evidence, final output,
   cancellation, and deterministic completion or explicit failure.
2. Finish soft/strict area-based mass preference and lazy native Detour transfers
   with checked query anchors and a bounded per-query cache.
3. Add the narrow opt-in minimum-area policy with reasons and identity. Complete
   serialization, diagnostics, and dirty-mesh regression fixtures. Reapplying a
   policy that restores unsampled islands requires discovery for those islands;
   retained metrics alone cannot recover omitted crossings.
4. Keep trusted seeded mode tested, but do not require seeds for MVP. Integrate a
   host validator only when collision or equivalent execution evidence exists.
5. Benchmark later per current user direction. Then resolve measured blockers and
   complete host migration, diagnostics/UI, cache replacement, and package version.

Deferred: universal collision engine or physical jump solver, authoritative
automatic interior classifier or sink pruning, built-in shape/statistical
classifiers, auto-seeded geometric subset builds, multiple movement profiles per
artifact, spans and approximate pruning, regional routing and same-island shortcuts,
incremental tile builds, artifact disk caches, cross-query transfer caches, worker
parallelization, and new dependencies.

Reconsider spans if measured candidate volume defeats bounded exact-only processing.
Reconsider regional routing if portal expansion/native transfers miss latency
requirements. Preserve endpoint provenance now; require measured benefit and
explicit loss/validity guarantees before either redesign enters scope.
Benchmarking remains deferred, so production scalability remains an open acceptance
question. Do not pivot architecture again solely on unmeasured cleanup claims.
