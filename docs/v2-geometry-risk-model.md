# V2 geometry risk model

This document defines geometry and evidence risks for automatic traversal generation
in DetourIslandGraph V2 and Gw2CollisionExtractor. Use it when adding fixtures,
changing validation, or optimizing discovery and routing. It covers correctness and
resource exhaustion; security threats are outside its scope.

The current movement model is teleportation. A validated traversal checks the
destination capsule and center support against supplied collision triangles. It
does not check a jump arc or the path between endpoints.

## Inputs and responsibilities

Raw collision is a triangle soup extracted from game data or loaded from OBJ.
It can contain overlapping surfaces, hollow shells, missing geometry, and surfaces
with no gameplay meaning. Triangle presence alone does not identify playable areas.

Recast produces a `dtNavMesh` under recorded bake settings. Its walkable polygons
describe geometric walkability under those settings. They do not establish intended
accessibility, complete collision coverage, or permission to teleport there.

Gw2CollisionExtractor runs traversal generation after final ground-navmesh assembly,
while raw collision remains available. It owns collision evidence, coverage assessment,
and the execution-matched destination validator. DetourIslandGraph owns portable
discovery, graph compilation, routing, serialization, and health analysis.

The eventual host must match graph identity and movement policy to execution.
Host integration and in-game verification remain deferred. The current Extractor
CLI runs traversal analysis and diagnostics but does not deliver a persisted graph.

## Evidence boundaries

- Ordinary DAT/OBJ extraction does not establish complete collision coverage.
  Strict validation must not promote incomplete evidence to `Valid` merely because
  no obstruction was found. Known obstruction can still establish `Invalid`.
- Standalone fixtures can explicitly declare their supplied triangles to be the
  complete test world within stated bounds. This proves nothing about completeness
  of the original game map.
- A clear, supported destination can lie inside a hollow shell. Destination-only
  validation does not test closed-solid containment or intended playability.
- A boundary absent from the input cannot cause a geometry-based rejection.
  Such restrictions require additional evidence or an explicit application policy.
- Health describes consistency under recorded inputs and policies. It does not
  certify discovery completeness or gameplay correctness.

## Failure cases and current controls

| Risk | Observable failure | Current control or test | Remaining limit |
| --- | --- | --- | --- |
| Hollow interiors | A supported floor inside a shell receives a traversal. | Procedural hollow-shell fixture records acceptance under teleport semantics. | Clearance cannot label the interior unwanted. |
| Small islands and narrow platforms | Many tiny islands increase discovery and routing work; some are legitimate destinations. | Narrow-platform scene, sparse-island stress, and explicit resource budgets. | Size alone does not establish invalidity; automatic cleanup is not implemented. |
| Overlapping floors and low voids | A baked landing lacks headroom or support in supplied collision. | Destination capsule and center-support checks; low-ceiling and unsupported-landing fixtures. | Spacious unwanted voids may pass. Center support does not prove support under the whole footprint. |
| Vertically stacked layers | Horizontal overlap produces many nearby candidates or selects an unreachable layer. | Discovery climb/drop limits, collision volume filtering, exact collision tests, and stacked-layer fixtures. | Dense overlap still increases broad-phase and routing work. |
| Missing gameplay boundaries | A clear mountain top or other restricted surface receives a traversal. | Unrepresented-boundary fixture records that geometry cannot infer missing restrictions. | Needs boundary data or an explicit policy. |
| Incomplete collision | Missing triangles falsely appear to provide clear space. | Coverage assessment and strict `Unknown` handling; incomplete-clear and incomplete-obstructed scenes. | Complete original-game coverage remains unproven. |
| Coarse sampling | Discovery misses a narrow valid landing window. | Known-window fixture contrasts coarse and fine spacing. | Bounded exhaustive discovery covers its configured samples, not every continuous landing position. |
| Dense candidate sets | Build or query work grows beyond acceptable limits. | Fixed dense workloads, allocation/work limits, cancellation, and route counters. | Passing a budget does not establish acceptable production latency. |
| Graph or route defects | Wrong direction, missing adjacency, invalid anchors, or incorrect reachability. | Graph health, independent BFS, route-leg checks, analytic discovery expectations, and serialization/batch comparisons. | A structurally correct graph can encode insufficient or unsuitable input evidence. |

A 2D broad-phase index is compatible with stacked geometry when later stages apply
vertical bounds and exact 3D tests. Changing the index requires measured evidence
that broad-phase work is the bottleneck.

## What the fixtures establish

The [routing baseline](v2-routing-baseline.md) records independent minimum-cost
checks and measured work. Optimality applies to the supplied estimated cost model;
it does not establish actual on-mesh walking paths. Large stress graphs retain
reachability checks without the quadratic minimum-cost oracle.

Exact-arrival suppression applies only to built-in Euclidean costs without a
crossing filter. Identical island, polygon, and position give identical remaining
costs, so an equal or more expensive arrival needs no second outgoing scan. This
does not merge nearby anchors or remove crossings. Custom callbacks use the
reference search. Fixture queries compare both modes for reachability and cost;
large dense queries also enforce a tenfold scan reduction.

The real Pandora and Steelribs fixtures include OBJ collision, baked navmeshes, and
sanitized bake provenance. Extractor tests rebuild the navmeshes, use OBJ triangles
for collision, and exercise standalone-complete and incomplete evidence modes.
They provide realistic geometry and repeatable regression inputs.

Procedural fixtures provide expectations that real maps cannot supply on their own.
Independent rectangle calculations check candidate discovery. Controlled collision
scenes isolate support, obstruction, coverage, and teleport semantics. Sparse and
dense workloads exercise resource limits. These expectations must remain independent
of the production algorithms they check.

Relevant test sources in this repository:

- [Analytic discovery and stress fixtures](../tests/V2Adversarial.h)
- [Adversarial test cases](../tests/V2AdversarialTests.cpp)
- [Directed reachability and route oracle](../tests/V2RouteOracle.h)
- [Real fixture provenance and bake inputs](../tests/fixtures/real/README.md)
- [Public graph health contract](../include/detour_island_graph/v2/Health.h)

In Gw2CollisionExtractor, see `tests/CollisionScenarios.h`,
`tests/fixture_health.cpp`, and `docs/fixture-health.md`. Run
`scripts/test_fixture_health.ps1 -DigSource <DetourIslandGraph checkout>` there to
build and test both repositories and compare repeated reports. Add
`-Configuration Release` for Release measurements.

## Rules for future changes

1. Record the geometry, movement profile, bake settings, coverage declaration, and
   budgets needed to reproduce a failure. Avoid unexplained map-specific thresholds.
2. Classify the result as an implementation defect, missing evidence, sampling limit,
   resource limit, or policy decision before changing the algorithm.
3. Preserve independently checked candidates, directions, and route correctness when
   optimizing. Any intentional loss of connectivity needs a named, tested policy.
4. Measure discovery, validation, compilation, and routing separately. Current full
   scenario timings also include test-oracle, health, and serialization work; they
   must not be reported as production build latency.
5. Use deterministic work counters for regression limits. Record timings with build
   configuration and input identity; elapsed time alone is not a stable correctness gate.
6. Update this document and a fixture when a new failure class or guarantee appears.
   Keep completion status and benchmark snapshots in [V2 progress](../V2_PROGRESS.md).

Soft area penalties, minimum-area exclusion, and gameplay-region labels remain
policy work. Penalties can change route preference but do not by themselves reduce
candidate-generation work or identify all unwanted surfaces. Native on-island
transfer routing also remains deferred; current estimated transfer costs do not
establish actual walking paths or distances.

No measured percentage of unwanted navmesh surfaces is established. Map-specific
layer heights, island counts, and claims about earlier performance require recorded
inputs and measurements before they become project assumptions.
