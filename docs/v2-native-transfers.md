# Native V2 transfers

Enable `DETOUR_ISLAND_GRAPH_NATIVE_TRANSFERS=ON` to build the optional provider in
`NativeTransfers.h`. Repository test presets enable it. Existing consumers keep
the feature disabled by default. Detour and every consumer must use the same
`DT_VIRTUAL_QUERYFILTER` ABI. Fetched Detour receives the matching build option;
externally supplied Detour must already provide it. The native source fails to
compile without that definition. Defining it only on DIG does not repair an
incompatible prebuilt Detour library.

`findNativeRoute` accepts explicit start/end polygon anchors, the frozen final
navmesh, its caller-supplied mesh identity, native limits, and ordinary route
options. It checks identity equality, polygon ownership, included domain, finite
positions, and projection tolerance. Zero identity supports same-session inputs
under the frozen-snapshot contract; it is not proof that arbitrary meshes match.
No nearest-polygon or nearest-layer selection occurs.

The provider uses a virtual filter to restrict search to ground polygons owned
by the requested island. This preserves build-time polygon exclusions and rejects
all off-mesh actions. It never changes mesh flags. The filter intentionally uses
graph ownership rather than default Detour flag masks.

## Cost and failure contracts

Transfers run lazily inside portal search. Sliced Detour search produces a complete
corridor; straight-path extraction includes all polygon crossings. Cost sums 3D
segment lengths in navmesh units. This is a navigation model, not exact detail
surface walking distance or a continuous-space shortest-path guarantee. The
provider checks projections but does not change stored crossing anchors or their
collision validation evidence. Callers must calibrate acceptable displacement.

Native transfers are labeled non-estimated model costs; default crossing distance
remains estimated. Portal search uses Dijkstra and disables arrival dominance for
this slice. Same-island queries retain `SameIsland` without a route value, after
endpoint checks. Direct native transfers remain available through the provider.

Typed transfer results distinguish blocked paths from fatal errors. Exhaustive
unreachability blocks one transfer. Node exhaustion, iteration/query limits, or
truncated corridor/corner buffers abort routing with `BudgetExceeded`; a partial
Detour result never becomes a successful transfer. Invalid input, cancellation,
allocation failure, and callback exceptions also abort without a route value.
Legacy float transfer callbacks and Euclidean routing remain available. Supplying
both transfer callback forms is invalid.

All native work/buffer limits are explicit and positive. Node capacity must be
4..65535; smaller values are unsafe for the upstream node-pool hash table. Changing
node capacity recreates the query because Detour otherwise retains larger pools.
Sliced searches check cancellation at most every 32 expansions and while checking
corridors and costs. Allocation and individual Detour calls are not interruptible.

Cache keys preserve ordered exact island/polygon/position identity. Successful and
blocked results are cached up to the entry cap; a full cache stops inserting.
Zero capacity disables caching. Every route resets entries and counters; buffers
retain capacity. One provider and route scratch per concurrent query. Frozen mesh,
graph, and callback captures must outlive provider evaluations. No cross-query
result reuse, mesh copy, new serialization, or host integration is introduced.

## Fixture verification

Pandora and Steelribs run every ordered included-island pair with caching enabled
and disabled, before and after decode, across build batches. A separate unsliced
Detour evaluator validates corridors and computes a transfer-cost table for the
independent portal-search oracle. Selected route legs and their costs are checked
again against that model. Both evaluators share upstream Detour geometry; this
does not independently prove Detour itself correct.

Fixtures use projection tolerance 0.60 navmesh units. Measured coarse-anchor to
detail-surface displacement reaches approximately 0.20 in Pandora and 0.55 in
Steelribs. This explicit test allowance is not a production movement calibration.
Reports include maximum observed displacement, query/iteration counts, cache
hits/misses/occupancy, blocked results, corridor/corner peaks, and route timings.
Timings remain separate from deterministic comparisons.

Procedural tests cover U-shaped detours changing crossing selection, off-mesh
shortcut rejection, excluded bridges, query/node/iteration/buffer budgets,
cancellation, anchor errors, identity mismatch, typed failures, and scratch reuse.
Host execution and collision completeness remain outside these checks.

Validation on 2026-09-08: 130 DIG tests and all five Extractor suites pass in
MSVC Debug and Release. Repeated reports and independent source hashes match.
The feature-disabled Release build and 16 routing regressions also pass.

Release standalone batch-1/original measurements, including query setup and
excluding oracle work:

| Fixture | Ordered pairs | Successful cross-island pairs | Native searches | Cache hits | Total routing ms |
|---|---:|---:|---:|---:|---:|
| Pandora | 625 | 4 | 1,303 | 972 | 1.11 |
| Steelribs | 225 | 22 | 104,320 | 419,527 | 107.77 |

These are measurements, not latency thresholds. Strict standalone graphs visit
anchors displaced by up to 0.164 and 0.434 units respectively; the larger 0.20
and 0.55 maxima above include all geometric candidates in the DIG fixture graphs.
