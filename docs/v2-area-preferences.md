# Area-based route preferences

`RouteOptions::areaPreference` changes query ranking without rebuilding or changing
the graph. It works with Euclidean costs, custom callbacks, and `findNativeRoute`.
It requires no collision input. Both controls default off.

```cpp
RouteOptions options;
options.areaPreference.preferredAreaSquareMeters = 100;
options.areaPreference.maxEntryPenalty = 10;
options.areaPreference.minimumIntermediateAreaSquareMeters = 25;
```

These values are fixture regression inputs, not calibrated production settings.
`maxEntryPenalty` uses the same units as movement costs. Built-in Euclidean and
native corridor costs use navmesh length units. Callers using time or other custom
costs must supply a penalty in those units.

## Semantics

Area is stored coarse polygon surface area divided by `unitsPerMeter` twice.
It is independent of polygon count, other islands, and detail-mesh relief.
Each intermediate entry adds
`maxEntryPenalty * max(0, 1 - area / preferredAreaSquareMeters)`.
The strict minimum rejects entries below its threshold; equality is allowed.
Start and destination islands are always exempt, including revisits. Repeated
entries into other islands pay again. Soft preference never removes a traversal;
strict preference may intentionally return `NoPath`.

All fields must be finite and nonnegative. A positive penalty requires a positive
preferred area. A preferred area alone does not activate policy. Active policy
requires metrics for every island, finite nonnegative areas, and positive finite
units with representable area conversion. Missing or invalid data returns
`InvalidInput`, including same-island queries. Disabled policy accepts graphs
without metrics. Existing endpoint/domain validation still applies.

Strict checks precede user filters and movement evaluation. Existing filters and
costs remain effective; penalties add to valid crossing costs. Active controls
use Dijkstra and disable arrival dominance. Disabled controls retain existing
search and work counts. Metric validation is O(islands), with cancellation checks;
per-entry policy evaluation is O(1), with no extra query allocation or persistent
cache. Policy arithmetic overflow returns `InvalidInput` without a route.

`Route::totalCost` includes preference penalties and is an optimization objective.
`Route::areaPenaltyCost` reports the selected route's penalty subtotal. Movement
cost provenance remains unchanged; penalties cannot turn estimated movement
into measured movement. Changes add C++ fields; rebuild consumers. Graph and nav
serialization formats are unchanged.

## Validation and limits

Synthetic tests establish competing-route ranking, strict rejection, threshold
equality, endpoint exemptions, repeated entries, units, unrelated islands, callback
composition, invalid input, cancellation, and scratch reuse. Independent oracle
code checks selected objective costs and penalty subtotals.

Pandora and Steelribs run every ordered included pair with neutral, soft, strict,
and combined settings under Euclidean and native costs. Profiles use preferred
area 100 square meters, maximum penalty 10 cost units, and strict minimum 25 square
meters. Checks include native caching, serialization, discovery batches, and
soft/strict reachability invariants. Extractor reports settings, changed routes,
lost routes, costs, penalties, and work counters. Timing remains separate from
deterministic comparisons. Equal-cost tie changes can count as changed routes.

Area does not prove local width, clearance, safety, collision completeness, or
intended playability. Host calibration and build-time domain heuristics remain
separate work.
