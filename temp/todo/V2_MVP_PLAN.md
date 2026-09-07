# DetourIslandGraph V2 MVP

Revised 2026-09-07 after dirty-map and routing reassessment. This document controls
MVP scope. [V2_PROGRESS.md](../../V2_PROGRESS.md) records delivered code and gaps.
Requirements below are not claims of implementation.

Current execution scope: library implementation first; benchmarking is deferred
by user request. Host migration and its acceptance gates remain future work.

## Direction

Keep V2's directional crossing model and immutable graph compiler. Expand MVP to
separate native topology, playable-domain selection, traversal validation, and
route preference. Dirty collision-derived navmeshes are a primary workload.

An island is a native connectivity group. Size does not prove playability, safety,
clearance, or short travel between portals. Final navmesh alone cannot reliably
distinguish legitimate caves from unwanted hollow interiors.

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

## 2. Exhaustive and seeded build modes

Keep exhaustive mode as the exact-only reference and for unseeded workloads.
Add explicit seeded mode for contaminated maps:

1. Resolve verified player-ground anchors or checked landmarks onto eligible
   polygons. Check vertical layer and projection tolerance. A POI coordinate alone
   is not trusted ground. Reject ambiguous or unresolved required seeds.
2. Activate seed islands and sample their boundaries at configured spacing.
3. Discover nearby destinations, including inactive eligible islands.
4. Validate each eligible direction. Activate a destination only through a Valid,
   policy-allowed outgoing traversal from an active island.
5. Process newly active islands until frontier exhaustion. Reverse-only acceptance
   does not expand forward reach. Preserve AB and BA results independently.

Seeded expansion requires a supplied host validator. Unknown directions do not
activate islands. Geometric-only experiments remain explicit exhaustive builds;
they do not establish a trusted seeded domain.

Unseeded components remain outside the declared build domain, not confirmed junk.
Distinguish out-of-domain query endpoints from no route inside the domain. Persist
seed identity, mode, exclusion policy, coverage, and completion state. Completion
means frontier exhausted under configured sampling, producer, and movement model,
not continuous-space or whole-map completeness.

This reduces disconnected garbage-to-garbage work only with reliable seeds and
validation. Initial topology extraction and nearby-target queries still cost work.
Huge active islands remain expensive. Erroneous native links or extraction holes
can admit junk; those need upstream exclusions or geometry repair. Missing seeds
or movement types can omit legitimate areas. These limitations must be visible.

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

Specify sample, candidate, temporary-query, and memory/work budgets. Sample and
candidate caps alone do not bound topology or nearby-polygon allocations. Exactly
the cap is allowed; exceeding it returns `BudgetExceeded`, counters, and no
publishable graph. Never thin samples silently or publish an interrupted frontier
as complete. Cancellation and callback failures abort publication through all
stages. Failed rebuilds preserve the previous compatible immutable graph.

## 4. Host traversal validation

Ship one working host validator matching actual gap execution. A callback contract
alone is insufficient. Inventory available collision and landing data before
choosing implementation; missing evidence stays Unknown, not Valid.

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
- Validated-only compilation accepts Valid directions and requires a validator.
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
reasons, seed resolution, frontier completion, spacing, queries/projections, exact
duplicates, directional outcomes, graph size, stage timing, and peak memory.
Separate exclusion from validated unreachability and build failure. Expensive
nearest-representative diagnostics remain opt-in.

Fixtures and focused checks must cover:

- Valid tiny platforms, large sealed interiors, legitimate caves, stacked layers,
  ambiguous seeds, one-way destinations, and erroneous native connectivity.
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
full host migration. Hold capabilities fixed; compare exhaustive reference with
seeded production builds. Measure false links, lost valid exits, labeled-domain
coverage, actual route cost/execution outcomes, build time, memory, and query
latency distributions. Dense sampling is a reference, not continuous-space ground
truth. High connectivity alone is not acceptance.

Choose performance thresholds from host requirements and measured baselines. No
arbitrary one-second target or 100% connectivity requirement. Run relevant library
tests, host dependency/targeted compilation checks, then real routes, rebuild and
cancellation, and stale-cache checks. Record unavailable in-game/full application
checks explicitly; do not mark them passed.

## 8. Remaining implementation order and exclusions

1. Capture dirty-map fixtures and baseline; document collision access, trusted
   anchors, host execution semantics, and route cost units.
2. Separate topology/sampling; add metrics, domain decisions, exclusion propagation,
   coverage contracts, and semantic identity.
3. Integrate host validator, seeded frontier mode, and bounded candidate processing.
4. Integrate mass policy and lazy native transfers; fix cost provenance and search
   termination. Extend serialization for settled contracts.
5. Benchmark both workloads, resolve measured blockers, then complete host migration,
   diagnostics/UI, cache replacement, and package version change.

Deferred: universal collision engine or physical jump solver, automatic interior
classifier or sink pruning, multiple movement profiles per artifact, spans and
approximate pruning, regional routing and same-island shortcuts, incremental tile
builds, artifact disk caches, cross-query transfer caches, worker parallelization,
and new dependencies.

Reconsider spans if measured candidate volume defeats bounded exact-only processing.
Reconsider regional routing if portal expansion/native transfers miss latency
requirements. Preserve endpoint provenance now; require measured benefit and
explicit loss/validity guarantees before either redesign enters scope.
