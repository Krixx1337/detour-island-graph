# DetourIslandGraph evolution recommendations

Date: 2026-09-07. Analysis only; no implementation changes.

## Decision

Evolve toward a dedicated, geometry-aware traversal build stage followed by a separate global graph compiler. Run that stage immediately after navmesh construction when possible, while collision and bake metadata remain available. Keep final-navmesh post-processing as a supported input path, not the only architecture.

Preserve symmetric-first corridor storage. Replace the assumption that endpoint limits prove traversal validity, and gradually replace global sampling budgets with spatial coverage controls. Improve on-island transfer costs before making stronger pruning claims.

This is an architectural separation, not a recommendation to rewrite everything or move work onto the game thread. Existing builder entry points can remain convenience wrappers.

## Evidence and limits

Evidence labels used below:

- E1: [Comparative audit](COMPARATIVE_AUDIT_UNITY_UNREAL.md), especially sections 2, 4, and R1-R10. Its source-derived descriptions are evidence of techniques used elsewhere, not proof those techniques improve this library under its workloads.
- E2: [Queensdale report 5](temp/queensdale5.txt). Mainland has 240 representatives, nearest-representative distance p95 100.62 and maximum 177.07, and midpoint uncovered fraction 70.19%. Mainland-connected geometry covers 51,069 of 52,541 polygons. These are observations for one configuration, not a benchmark or ground-truth recall measurement.
- E3: Local implementation inspected in [boundaries](src/IslandGraphBoundaries.cpp), [scanner](src/IslandGraphScanner.cpp), [pruning](src/IslandGraphPruning.cpp), [pathfinder](src/IslandGraphPathfinder.cpp), [builder](src/IslandGraphBuilder.cpp), and [graph types](include/detour_island_graph/IslandGraph.h).
- E4: [Epic automatic navigation-link documentation](https://dev.epicgames.com/documentation/unreal-engine/automatic-navigation-link-generation?lang=en-US). Independently checked. Generation occurs during tile generation; longer jumps can increase rasterization size; sampling separation trades speed for precision. Documentation currently labels the feature Experimental.
- E5: [Detour query documentation](https://recastnav.com/classdtNavMeshQuery.html). Independently checked. Raycast is a surface walkability query that ignores endpoint height. Straight-path generation operates inside a supplied polygon corridor.
- E6: Local Unreal source, `E:/Projects/UnrealEngine-release/Engine/Source/Runtime/Navmesh/Private/Detour/DetourNavLinkBuilder.cpp`. Relevant ground-sampling, heightfield collision, overlap-filter, and trajectory symbols were located. Detailed algorithm descriptions below rely on E1, not a fresh exhaustive Unreal source review.

Unity details are attributed to E1 and its [Unity manual source](https://docs.unity.cn/Manual/nav-BuildingOffMeshLinksAutomatically.html). Direct manual retrieval failed during this review. No claim of independent re-verification of those details.

All proposed architecture, priority ordering, and expected performance benefits below are engineering inference. No optimized comparison builds or execution trials were performed. The mistakenly referenced `research.md` is not an input.

## Corrections to the comparative audit

1. R1's suggested Detour raycast/straight-path validator is not an airborne collision validator. E5 explicitly limits those queries to navmesh traversal. A missing walkable surface under a jump is expected; a wall hit in that surface query is not a reliable physical-flight result. Use collision geometry or solid-heightfield access for flight clearance.
2. The current scanner checks endpoint geometry, not even collision along a zero-width segment. Its capability predicate should be described as geometric eligibility, not trajectory validation. E3.
3. R10's prohibition on bake-time generation is too strong. Local traversal generation and global graph compilation can occur at different times. Locally generated corridors can feed global connectivity and routing. This is the basis of the recommendation here.
4. Unity's restricted automatic generation modes do not establish that Unity links cannot be one-way. E1's general statement about inability to express one-way routes is not supported by its description of automatic drop links. Likewise, engine-wide claims about absence of hierarchical routing or inferior diagnostics exceed the reviewed feature scope.
5. UE's overlap test compares takeoff/landing spans. Applying it directly to point-to-point flight segments changes its meaning. Nearby flight lines need not have interchangeable endpoints or approaches.
6. A lossy voxel first pass cannot be repaired by a more accurate second pass. Use the spatial grid to find possible duplicates, then apply an explicit equivalence/dominance test before discarding geometry.
7. Mass-aware budgets are not an unconditional strength. E2 directly motivates revisiting them. Strong aggregate connectivity can hide missing convenient exits.
8. Clearance inherited from an agent-specific navmesh is useful, but does not establish airborne clearance or compatibility with a different agent shape. Do not imply every projected endpoint is invalid, or that existing polygon filtering is absent.

## Highest-value changes

### 1. Separate geometric eligibility from validated traversal

Evidence: E1 describes endpoint ground sampling and collision-aware trajectories in engine generation. E3 accepts proximity-compatible endpoints. These answer different questions.

Recommendation: introduce a traversal validation contract with three outcomes: valid, invalid, and unknown. Record reason, movement-profile identity, environment revision, and validation method. Evaluate AB and BA independently.

The contract should accept immutable geometry access, agent dimensions, endpoint polygon references, and a movement model. A built-in ballistic model is one adapter, not the definition of every corridor. Drops, climbs, authored actions, and teleport-like traversal have different requirements. The host's existing movement contracts explicitly include teleport-based gap execution; forcing every current corridor through a parabola would change its semantics rather than simply fix it.

For physical jumping, require supported takeoff/landing, clearance for the agent volume, a feasible trajectory, and collision validation at a documented resolution. Prefer swept-volume queries where available; otherwise use conservative discretization tied to obstacle resolution and trajectory curvature. Fixed sample counts can miss thin obstacles. Validation still needs execution trials against the actual controller.

Keep legacy geometric-only mode explicit. A physical-jump routing policy must not silently treat unknown as validated. Missing collision data should yield unknown or a clear build error when physical validation is required, not an automatic success.

Run cheap bounds and policy checks first. Validate before destructive approximate deduplication and pruning, or retain alternatives until a validated representative wins. An obstructed top-ranked sample must not erase a nearby clear sample.

Cost: geometry integration and validation can dominate build time. Benefit: fixes false-positive semantics at their source and makes precision measurable. Highest priority for physical-jump consumers; for the current non-ballistic host, establish the contract first and prioritize sampling next.

### 2. Make sampling spatially bounded, not island-size-budgeted

Evidence: E2 shows severe mainland undercoverage. E3 samples boundary midpoints and applies representative reduction. E1 describes along-edge sampling and valid spans.

Recommendation:

- Extract true exposed edge intervals, including uncovered portions of partially linked tile portals. Preserve polygon, tile, and interval provenance. Distinguish unloaded seams from confirmed exposed boundaries in streaming worlds.
- Sample by boundary arclength with explicit world-space spacing and endpoint handling. Use bake resolution and agent dimensions when available, but do not derive physical sampling spacing from the deduplication grid.
- Preserve local sampling density across large islands. Budget exhaustion should report incomplete coverage, not silently lower density throughout the mainland.
- Use spatially local refinement around landing opportunities, height changes, and validation transitions. Adaptive refinement must retain a baseline spacing; it is not a completeness guarantee between samples.
- Separate horizontal sampling, vertical layer separation, landing tolerance, and capability limits. Increasing drop range must not coarsen horizontal discovery.

Do not add another multiplier on top of every existing suppression stage. Introduce one primary spacing policy, explicit resource limits, and measured approximation modes. Retire redundant controls after comparison.

A near-term experiment is a larger mainland representative budget at unchanged capabilities and pruning. It can establish value before the interval redesign. The eventual design should not contain Queensdale-specific exceptions.

Cost: more candidates. Benefit: less sensitivity to island size and triangulation. Quantitative benefit remains unmeasured. E2's uncovered fraction measures same-island midpoint proximity, not missing-link rate; reverse discovery from satellites can compensate.

### 3. Separate traversal generation from graph compilation

Evidence: E3 interleaves topology, mass policy, candidate discovery, and pruning in one build. E1 describes local generation using richer bake data. Inference: separating their artifacts improves reuse and allows either input source without duplicating graph logic.

Proposed flow:

```text
Navmesh + immutable geometry/bake context
             |
    local candidate generation
             |
    directional validation
             |
    reusable corridor artifact
             |
    global island/region graph compilation
             |
    query costs, filters, and runtime execution checks
```

The reusable artifact stores endpoint polygon anchors, optional paired spans, directional movement variants, validation provenance, and local dependency bounds. Do not persist mutable pointers. Raw Detour polygon references need mesh identity and remapping or rejection across rebakes.

Keep topology labels independent of candidate ownership. A corridor can be discovered before its final global island IDs are known. A compatibility wrapper can still perform all steps from `dtNavMesh` for existing callers.

Policy-only cost changes should not force collision resampling. However, changing agent radius or filters can change the navmesh or island partition; reuse is conditional, not universally valid. A union graph cannot safely represent every agent through scalar gap thresholds alone.

### 4. Preserve symmetric geometry, add directional action variants

Evidence: E3's `Edge` stores one endpoint pair with independent flags. This remains appropriate. E1's multi-config/per-direction patterns motivate richer permissions.

Recommendation: one canonical corridor geometry, with AB and BA records referring to validated action variants, costs, and requirements. Canonical endpoint order is storage order, never execution order.

Equal climb/drop limits do not prove equal trajectory feasibility. Different launch models, takeoff clearance, landing constraints, and policies can make a geometrically symmetric pair directional. Pair-scan suppression must eventually depend on validated equivalence, not equality of two limits.

Deduplication must preserve both endpoint regions, directional permissions, action class, and validation quality. Do not OR permissions from different geometry. Do not merge a cheap drop with an expensive climb just because both connect the same islands.

Store execution identifiers as passive data. The library must not own animation, physics, game callbacks, or movement authority. Keep behavior execution in the host's Mechanics layer.

### 5. Fix the meaning of transfer costs and pruning guarantees

Evidence: E3 uses Euclidean on-island transfers in both portal routing and two-hop pruning. A connected island guarantees some path under its construction filter, not a straight or short path between every portal pair.

Recommendation: treat Euclidean transfers as lower-bound estimates where the cost model permits, not proof that an alternate route is cheap. A large island can wrap around walls, ravines, or a long U-shaped obstacle.

For conservative pruning, require a feasible alternate path with measured cost or a valid upper bound, including start/end transfers and matching directional/profile permissions. If that evidence is unavailable, retain the corridor or clearly label pruning lossy. A Euclidean two-hop comparison does not establish a Detour-path stretch bound.

For routing, add a local transfer provider and bounded cache keyed by mesh revision, filter, cost policy, and endpoint anchors. Refine candidate routes with actual Detour transfers and reconsider alternatives when refinement changes ranking. Refining only the winning approximate route does not prove optimality.

Avoid eager all-pairs transfer tables for mainland portals. Start with lazy evaluation. If large-island routing remains expensive, partition routing into bounded local regions with portals while retaining islands as connectivity/diagnostic groups. This is a major optional redesign with more storage and invalidation complexity, justified by route-quality or latency measurements, not mainland size alone.

Also define scope explicitly: current inter-island discovery excludes useful jumps between two parts of the same island. That is acceptable for a connectivity bridge library, but incomplete for a general automatic jump-link generator. A future regional router can support those shortcuts without pretending they change island connectivity.

### 6. Use spans and local work to reduce overhead

Evidence: E1 describes contiguous valid spans and local filtering. E3 repeatedly projects nearby polygons and stores point candidates. Expected efficiency gains below are inference.

After along-edge validation works, group compatible neighboring samples into paired takeoff/landing intervals. Do not bridge invalid gaps or interpolate across different layers. Initially materialize selected center/extremity points for the existing router; retain span provenance so pruning does not erase distinct access locations.

Use spatial broad-phase queries over full expanded bounds, then exact checks. A midpoint-only segment index can miss overlapping long spans. Preserve existing Detour polygon queries as a backend and benchmark edge/landing-region indexes before replacing them.

Cache sort keys once, use contiguous adjacency and precomputed portal offsets, and reuse query scratch storage with clear per-worker ownership. Under a documented admissible heuristic and nonnegative cost contract, terminate A* when the remaining lower bound cannot beat the best goal. Arbitrary custom callbacks require a separate conservative contract.

Measure diagnostics separately. Current nearest-representative diagnostics perform sample-by-representative comparisons; a 32-island display cap does not bound cost on a huge island. Keep detailed coverage analysis opt-in or indexed, not an unconditional production build tax.

Do not add Taskflow, nanoflann, Morton indexing, or a new hash library as prerequisites. Revisit only after optimized stage timings, allocations, and candidate counts identify the remaining bottleneck.

## When should automatic links be generated?

These choices concern scheduling and available inputs. Post-processing is not inherently geometry-blind, and a dedicated stage can still run after baking.

| Approach | Main advantage | Main cost or limitation | Best fit |
| --- | --- | --- | --- |
| Inside each navmesh tile bake | Uses transient contours, solid geometry, resolution, and agent settings without reconstructing them | Couples link settings to tile work; reach may enlarge borders; cross-tile ownership and invalidation need care | An engine owning its bake pipeline with fixed agent profiles and frequent local tile rebuilds |
| Dedicated traversal build stage after navmesh generation, before discarding geometry | Separates traversal iteration from polygon baking; retains rich validation inputs; feeds global compilation | Requires an explicit geometry lifetime/artifact contract, dependency tracking, and its own cache | Recommended default for DetourIslandGraph evolution |
| Current final-navmesh-only post-process | Portable; works with imported Detour assets; global topology available; no baker integration | Final mesh does not preserve all collision solids, contours, or bake metadata; geometry-only eligibility cannot certify flight | Compatibility path, geometric connectors, or consumers supplying a separate collision provider |

Recommendation: the dedicated stage. If the host controls baking, invoke it adjacent to the bake while its data is live. If not, run the same stage against a frozen collision-mesh provider and loaded navmesh. If only a navmesh exists, produce explicitly geometric-only artifacts.

Prefer retaining compact boundary/provenance data and reusable collision access over keeping every raw heightfield resident indefinitely. If collision data must be reconstructed, measure its cost; the dedicated-stage advantage may shrink. Bake-time producers should remain valid alternative frontends, especially where retaining geometry would be more expensive than generating links immediately.

This rejects E1 R10's false choice between local generation and global topology. Global graph compilation follows either producer. The recommended split does not require injecting generated links back into Detour. An optional off-mesh export adapter can do that; island flood fill must still distinguish native walk connectivity from generated actions to avoid collapsing the abstraction.

## Incremental rebuilds and caching

Inference: local traversal artifacts offer the strongest long-term build-time improvement when maps or capability profiles change repeatedly. For static one-shot maps, full builds may remain simpler and adequate.

Cache local geometry work by tile/chunk identity, geometry revision, agent shape, movement model, sampling settings, and validator version. Cost-only routing profiles should use a separate identity. Include units and bake resolution where relevant; callbacks need caller-provided semantic versions or must disable reuse.

Invalidate every corridor whose trajectory/landing dependency bounds intersect changed geometry, including neighboring tiles within the search/trajectory envelope. Endpoint tiles alone are insufficient. Handle removals as well as additions.

A local tile change can split or merge a global island. Reuse local corridors but initially recompute global connectivity rather than implementing dynamic graph connectivity prematurely. Remap endpoints, reject stale references, then publish one compatible immutable graph. Reuse the host's maintenance lane, cancellation, atomic cache replacement, and revision checks; no new task manager.

## Delivery order and decision gates

1. Establish semantics and baseline. Distinguish geometric connectors from physical jumps; specify validation outcomes and directional records. Time discovery, validation, graph compilation, diagnostics, and queries separately. Keep current defaults compatible but accurately named.
2. Run the Queensdale sampling comparison. Hold limits and pruning fixed; increase mainland coverage. Measure recovered neighbors/exits, disconnected components, route detours, and cost. Do not aim blindly for 100% island connectivity.
3. Implement exposed intervals and arclength sampling, with explicit incomplete-budget reporting. Add the validation provider alongside this work when physical jumping is in scope. Compare against dense sampled references, which themselves are not continuous-space ground truth.
4. Separate corridor artifacts from graph compilation. Add provenance and cache versioning before multi-profile reuse. Remove unsupported pruning guarantees and introduce actual transfer refinement.
5. Add validated spans and local caching where measurements justify them. Retire redundant heuristic knobs instead of accumulating new ones.
6. Consider routing-region partitioning and incremental generation only after full-build/query baselines show their value. Parallelize bounded local work only with immutable inputs, worker-local Detour queries, deterministic merging, and cooperative cancellation.

Validation gates for implementation, proposed here rather than executed:

- Known clear and blocked crossings, thin flight obstacles, ceilings, narrow endpoints, and stacked layers.
- Unequal climb/drop limits, directional trajectories, outbound-policy differences, and several action variants on one corridor.
- Long-edge endpoint opportunities, partial tile portals, changed tile order, translation, scale changes, and equivalent retessellation. Expect bounded geometric error, not identical raw sample IDs.
- A mainland with large internal detours, same-island shortcuts if supported, and query filters that invalidate nominal island transfers.
- Cached artifact rejection after changed geometry, agent, validator, or units; cancellation must never publish mixed generations.
- End-to-end execution outcomes, false-positive rate, reference recall, actual route-cost stretch, build/query latency distributions, and peak memory. Choose acceptance thresholds from host requirements, not arbitrary universal health scores.

## Bottom line

The most valuable redesign is to separate where traversal geometry comes from, whether an action is valid, and how a global router uses it. Keep the symmetric-first model and portable post-process entry point. Add geometry-aware validation without hardwiring ballistic movement, replace island-global sampling quotas with local coverage, and stop using straight-line transfer estimates as preservation proofs.

Build traversal artifacts in a dedicated stage with rich geometry access. Compile global topology afterward. This offers a path to better robustness and reusable local work without binding the library to one engine's baker or committing to a wholesale rewrite.
