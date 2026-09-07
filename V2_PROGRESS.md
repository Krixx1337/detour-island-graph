# V2 MVP implementation status

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
  adjacency offsets, and polygon-to-island lookup. Routing portal-state layout
  is still pending.
- Input checks for finite geometry/settings, topology ownership, canonical order,
  directional consistency, and conflicting duplicate results.
- Cancellation and callback failure return no output, even after partial work.
  Candidate cap applies to raw input before exact deduplication. Exactly the cap
  is allowed; exceeding it fails the stage. Sample caps await sampling.
- Persistent-reuse eligibility requires versioned mesh/profile and any custom
  policies/validator/environment. This is an eligibility flag, not a cache-key
  implementation or serializer.

V2 currently lives in a separate namespace while replacement proceeds. There are
no V1-to-V2 compatibility wrappers. Existing host still uses V1. Public package
version and cache formats remain unchanged until the complete replacement lands.

## Trust and ownership contracts

Topology and candidates are trusted producer inputs, not serialized or hostile
input. The compiler checks consistency against the supplied polygon ownership
table, but cannot prove polygon references or endpoint positions belong to an
actual navmesh. The forthcoming mesh producer must establish that fact.

Callbacks must be deterministic and capture frozen state. Artifacts must not be
mutated concurrently with compilation. Validation results and their provenance
must remain paired; changing identity fields cannot revalidate geometry.

Teleport consumers can select geometric-only compilation. Unknown means no
confirmed traversal validation, not a collision-clear or ballistic jump claim.
Movement execution remains outside this library.

## Verification

- Targeted Windows/MSVC Debug library test build passed.
- All 46 tests passed: 35 existing V1 tests and 11 V2 contract tests.
- Root architecture boundary checker and whitespace checks passed.

No host switch, full application build, Queensdale performance measurement, or
in-game execution check is part of this foundation slice. No UE implementation
was copied; these contracts require no Unreal-specific algorithm.

## Next slice

1. Extract native ground topology and exposed edge intervals from frozen Detour
   input. Subtract the union of eligible external portal intervals.
2. Sample endpoints and interiors with explicit maximum spacing, deterministic
   ownership, cancellation, and fail-on-exceeded sample cap. No island quotas.
3. Discover projected anchored candidates with cancellable Detour queries and
   feed the validation/compiler stages. Enforce candidate cap while generating,
   not merely after allocating the candidate list.
4. Benchmark the dense, exact-only pipeline on Queensdale before full host
   migration. Record sample/candidate/crossing counts, stage timings and peak
   memory. The current ordered-map implementation is a correctness baseline;
   profile its allocation and duplicate-key cost before optimizing it.
5. Implement V2 routing and caller scratch, new serialization/cache identity,
   then host/settings/report migration and package version 2.0.0. Benchmark query
   latency when the router is available.

Sampling, actual discovery, mass diagnostics, routing, serialization and host
migration remain unfinished. This is the first foundation slice, not a usable
end-to-end V2 navmesh builder.
