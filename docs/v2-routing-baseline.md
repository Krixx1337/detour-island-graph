# V2 routing baseline

Recorded 2026-09-08 with MSVC x64 Release, default Euclidean costs, and the
fixture-health runner. Timings are observations, not regression thresholds.
Concurrent Debug validation may affect elapsed times. Work counters are the
reproducible baseline. No search optimization is included in this slice.

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
