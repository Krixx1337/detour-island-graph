# V2 routing baseline

Recorded 2026-09-08 with MSVC x64 Release, default Euclidean costs, and the
fixture-health runner. Timings are observations, not regression thresholds.
Concurrent Debug validation may affect elapsed times. Work counters are the
reproducible baseline. The following historical tables record the reference search
before exact-arrival suppression; the comparison at the end records the new path.

## Workloads and results

Real fixtures use the committed OBJ/nav/job inputs and existing movement and
discovery settings recorded by the report. The table covers one original graph,
batch size 1, standalone triangle-world coverage, and every ordered included island
pair, including SameIsland and NoPath. Each sequence starts with fresh scratch and
reuses it between queries.

| Fixture | Traversals examined | Portals expanded | Portals queued | Build ms |
| --- | ---: | ---: | ---: | ---: |
| Pandora | 2,184 | 2,097 | 2,184 | 3.67 |
| Steelribs | 474,751 | 11,197 | 22,685 | 3.77 |

Input nav SHA-256:

- Pandora: `ead44d6c9f890fb17993f9fb691cc628778e275c24706a423dae2385b1fcfbcd`
- Steelribs: `59dfdaf14b1cdeb1a09e9746b6926728fde6d636e1f59ba08917f41c06a82d1e`

Compiled graph SHA-256:

- Pandora: `37a4847cb80aa8736806993aeaf9093d846c90921abdcf681d73093ce3f09b19`
- Steelribs: `f454652824c71f3721dc0c2be0ef36e9800d767db4d9eee368413df052f21b8e`

The `dense_256` procedural world has 256 overlapping 2-by-2 rectangles at integer
heights 0 through 255. Sampling spacing is 2, horizontal reach 0.5, and climb/drop
limits 256. It produces 130,560 crossings and 261,120 directed traversals.
Representative queries use polygon centers and the existing one-million expansion,
two-million queue caps. Original and decoded graphs each run the same four queries.

| Original graph query | Traversals examined | Expanded | Queued | Route ms |
| --- | ---: | ---: | ---: | ---: |
| 0 to 255 | 132,131,820 | 129,541 | 506,430 | 962 |
| 255 to 0 | 132,131,820 | 129,541 | 509,745 | 911 |
| 0 to 128 | 33,163,260 | 32,513 | 255,765 | 226 |
| 128 to 255 | 32,645,100 | 32,005 | 249,390 | 235 |

Each examined traversal evaluates both costs in this workload, with one additional
transfer evaluation for the final destination. These queries have zero stale heap
pops. The full scenario took about 5.14 seconds, including both graph query sequences,
discovery oracle, health, and serialization. It is not a build-latency measurement.

## Interpretation and next target

Repeated scans of all outgoing traversals on arrival islands dominate the measured
operation counts. The next optimization investigation should target those scans
and repeated cost evaluation, while preserving portal-specific costs, directions,
and arbitrary custom callback behavior. These measurements do not justify removing
islands or pruning connections. Heap cleanup is not supported as the first target
by this dense workload's stale-pop count.

## Correctness and reproduction

Run Extractor's `scripts/test_fixture_health.ps1 -DigSource <DIG checkout>` with
`-Configuration Release` or `Debug`. Reports record all source hashes, build
configuration, settings, per-route counters, and timings. Compare all fields except
the top-level `timings` array between repeated runs.

Real fixture routes now compare minimum cost against independent linear-selection
Dijkstra built directly from enabled crossings. Procedural all-pairs checks use
that oracle only at 512 directed traversals or fewer; larger cases retain independent
reachability and leg-cost checks. Tests also cover custom, zero, blocked, equal, and
asymmetric route choices. This checks optimality under the supplied cost model,
not native Detour walking distances or intended game accessibility.

`RouteStats` counters count attempts, including initial scans and built-in cost
evaluations. Filter rejection happens before cost evaluation. Exceptions and
cancellation preserve work already attempted. Heap pops include stale entries;
stale pops do not consume expansion budget. Added public fields require rebuilding
consumers; serialized graph format is unchanged.

## Exact-arrival suppression comparison

Recorded 2026-09-08 after Debug validation finished, with MSVC x64 Release and no
concurrent Debug test load. Both modes run against the same graph within the same
runner. Every optimized duration includes grouping preparation. These are observed
first-report timings, not latency gates; repeated reports agree on deterministic
fields. The input and compiled graph hashes above remain unchanged.

| Dense original-graph query | Reference scans | Optimized scans | Reference ms | Optimized ms |
| --- | ---: | ---: | ---: | ---: |
| 0 to 255 | 132,131,820 | 1,037,340 | 981.77 | 67.28 |
| 255 to 0 | 132,131,820 | 1,037,340 | 950.71 | 61.29 |
| 0 to 128 | 33,163,260 | 519,180 | 244.80 | 45.94 |
| 128 to 255 | 32,645,100 | 515,100 | 249.19 | 46.45 |

All four queries exceed the required tenfold scan reduction. The long queries
expand 1,017 arrivals instead of 129,541. Grouping produces 1,024 exact anchors and
uses 4,182,016 logical bytes, about 3.99 MiB, in addition to existing route scratch.
This excludes vector capacity and allocator overhead. Queue counts remain unchanged
in these dense queries because suppression happens when popping arrivals, after
they have been queued.

The real-fixture table covers every ordered included pair for original graphs,
batch 1, standalone triangle-world coverage. Durations are totals over each query
sequence, with separate scratch for each mode.

| Fixture | Queries | Reference scans | Optimized scans | Reference ms | Optimized ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| Pandora | 625 | 2,184 | 2,184 | 0.12 | 1.08 |
| Steelribs | 225 | 474,751 | 201,697 | 3.97 | 4.89 |

Preparation adds overhead to small queries. Steelribs examines fewer traversals but
still takes slightly longer overall; Pandora has no scan reduction. This optimization
addresses dense routing and is not a universal latency improvement. Callers can set
`enableArrivalDominance=false` to use the reference search. A future slice should
measure how to avoid preparation when little search work is expected, or reuse an
immutable grouping with an explicit graph-lifetime contract. No such cache exists
in this slice.

Suppression requires identical island, polygon, and numeric position, and a cost
at least as high as an arrival already expanded there. A strictly cheaper arrival
can expand again. Cost/filter callbacks disable suppression, preserving their
reference execution. Dominated pops consume no expansion budget and remain distinct
from stale heap pops. No crossing is removed and no approximate anchor merge occurs.

Validation: 126 DIG tests and all five Extractor suites pass in Debug and Release.
Real and procedural fixture queries compare reference and optimized reachability
and costs. Independent minimum-cost checks retain their existing size limit.
Repeated reports match outside timings, source SHA-256 checks pass, and within-mode
batch/serialization work counters match. Fixed dense scan ceilings enforce at least
tenfold reduction relative to the reference search.
