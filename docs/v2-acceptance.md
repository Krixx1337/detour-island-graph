# V2 automated acceptance

This audit covers DetourIslandGraph and Gw2CollisionExtractor before host migration.
The controlling scope is [the MVP roadmap](../temp/todo/V2_MVP_PLAN.md).
[Progress](../V2_PROGRESS.md) records dated validation results. Passing these checks
establishes the tested contracts, not complete game collision or movement safety.

## Reproduce

From an x64 Visual Studio Developer PowerShell in the Extractor checkout:

```powershell
./scripts/test_v2_acceptance.ps1 -DigSource <path-to-DetourIslandGraph>
```

The command runs DIG and all seven Extractor CTest suites in Debug and Release,
then DIG Debug with native transfers disabled in a separate build directory.
Tests are explicitly enabled; an empty CTest selection fails. Debug and Release
run serially because CLI jobs share the extraction mutex. Native fixture runs
repeat and compare reports within each configuration, excluding timings only.
Source SHA-256, nav/graph pairing, rebuild equivalence, and accounted budgets are
checked independently of the report producer.

Extractor `out/v2-acceptance/acceptance.json` records revisions, dirty-state flags,
completed checks, failures, and report locations. Build logs and reports stay in
ignored output directories. A dirty revision is not a reproducible source archive;
retain the corresponding diff when sharing a result. The script stops at the first
failure and records it. It does not commit, publish, or change dependency pins.

## Contract evidence

The audit traced the production bounded builder and its allocator through topology,
discovery, validation and compilation; serializer reconstruction; routing and native
transfer failure handling; and Extractor capture, provenance, decoding and atomic
publication. Reference discovery remains a small-input oracle, not the bounded
production API. Test names below identify executable evidence, not just code presence.

| Contract | Implementation and named automated evidence | Audit status |
| --- | --- | --- |
| Reciprocal native ground islands; off-mesh actions excluded | DIG `extractTopology`; `V2 sampling excludes off mesh actions even when polygon filter accepts everything`; `V2 sampling rejects invalid settings precision loss and one way native ground` | Covered |
| Metrics, explicit exclusion, retained ownership and reasons | DIG topology/compiler; `V2 topology metrics use 3D area and preserve polygon exclusions`; `V2 domain policy preserves ownership while gating sampling discovery and routing` | Covered |
| Partial portals, endpoint spacing and exact deduplication | DIG sampling; `V2 sampling subtracts every external portal interval and joins every eligible neighbor`; `V2 sampling unions overlapping portal intervals rather than emitting false gaps`; `V2 sampling includes long edge endpoints and bounds spacing without island quotas` | Covered |
| Scale, translation, retessellation and tile allocation order | `V2 sampling scales with explicit spacing and does not couple density to drop range`; `V2 sampling internal ground connectivity and filter boundaries survive retessellation`; `V2 sampling labels islands and owns exact duplicate samples independently of tile load order`; `V2 bounded production preserves partial portals under tile allocation order` | Covered for declared synthetic geometries, not arbitrary tessellation-invariant samples |
| Independent direction eligibility and finite sampling limits | DIG discovery/compiler; `v2 canonical geometry retains independently validated directions`; `V2 discovery keeps reverse-valid asymmetric pairs`; `V2 narrow landing window documents finite sampling coverage` | Covered |
| Bounded exhaustive production, batching and cancellation | `V2 bounded production permits exact resource caps and rejects next operation`; `V2 bounded every cancellation checkpoint releases unpublished graph`; `V2 bounded collector rejects dense stacked query before a 65th ref is stored`; `V2 real fixtures match reference discovery across batches` | Covered within documented accounting |
| Optional trusted seeds and completed frontier | `V2 seeded frontier follows only validated forward reach and persists coverage`; `V2 seeded reverse expansion is deterministic and respects exact caps and outbound policy`; `V2 bounded seeded frontier matches reference and budgets span islands` | Covered synthetically; real trusted seeds not required |
| Invalid/Unknown/Valid compilation and revision semantics | DIG compiler; `v2 unknown validation is explicit and compilation never calls validator`; `v2 compiler rejects forged permissions and conflicting exact duplicates`; `v2 unversioned custom semantics disable persistent reuse` | Covered |
| Serialization, old/corrupt data, counts and domain consistency | DIG serializer reconstructs through compiler; `V2 serialization rejects foreign corrupt and hostile input`; `V2 serialization rejects corrupt metrics and domain records`; deterministic round-trip test | Covered; decode allocation allowance is not a whole-process cap |
| Route optimality under declared costs, failure propagation and scratch reuse | DIG routing; `V2 independent minimum cost oracle covers custom and directed routes`; `V2 routing waits for competing arrival when native finish cost is expensive`; `V2 typed transfers preserve anchors and propagate fatal outcomes`; `V2 arrival preparation cancellation leaves reusable scratch` | Covered, including fixture route oracles |
| Native on-island corridors and bounded per-query cache | DIG native provider; `V2 native transfers follow corridors and enforce resource limits`; `V2 native filter rejects excluded polygons during search`; `V2 native detour changes the selected crossing`; fixture native-cost oracle | Covered; corridor length is not continuous-space optimality |
| Soft/strict route-area preferences and endpoint exemptions | DIG routing; `V2 area preferences rank routes and preserve endpoint exemptions`; `V2 area policies reject corrupt metrics and converted areas`; fixture area oracle | Covered; no playability inference |
| Structural health, directed reachability and meaningful failures | DIG health; `V2 health detects corrupt graph views without exposing mutation API`; `V2 health bounds analysis and handles cancellation`; Extractor fixture independent BFS and route checks | Covered; successful analysis still requires checking `healthy()` |
| Collision capture, transforms, coverage vetoes and teleport destination semantics | Extractor capture/snapshot/validator; `Collision capture preserves filtered geometry and bake counters`; `Coverage declarations are vetoed by capture defects`; `Teleport destination checks support and capsule rather than connecting path`; `Traversal aborts without graph when query budgets or cancellation trigger` | Covered for supplied triangles |
| Matched raw/protected delivery and bounded publication | Extractor bundle loader/writer; `Bundle reload preserves mesh and rejects damaged or incompatible files`; `Bundle publication preserves destination on failure and process termination`; `Bundle working budgets reject before publication and preserve exact boundaries`; `Self-consistent bundle counts are rejected before native construction`; codec-overlap test | Covered; atomic replacement is not an authentication or universal power-loss guarantee |
| OBJ/DAT rebuild provenance, fresh coverage and unchanged MSET | Extractor shared rebuild path; OBJ CLI regressions; `DAT parsing and link rebuild preserve matched bundles without Recast`; `DAT rebuild requested GUID requires actual matching PARM`; fixture full/rebuild equivalence with changed movement reach | Covered through OBJ files and injected decompressed DAT payloads |
| DAT failure preserves publication and allows retry | `DAT rebuild read parse and bundle failures preserve previous publication`, raw and protected cases | Coverage added by this audit |
| Portable configuration without native transfers | Separate Debug configure/build/CTest invocation in acceptance script | Executed by acceptance command; results recorded in progress |
| Built-in minimum-area build-domain policy | Caller `islandPolicy` supports custom exclusions; no built-in minimum-area helper exists | Deferred by user after audit; not an MVP blocker |

## Findings and remaining work

The complete acceptance command passed on 2026-09-09: 135 DIG tests in each native
Debug/Release configuration, seven Extractor suites in each configuration, and
132 DIG Debug tests without native transfers. Both real fixtures passed repeated
report, independent source-hash, bundle identity, rebuild and accounted-budget
checks. No host or in-game testing was run. These results cover dirty working
trees, including this audit's regression and script changes.

The audit adds one DAT regression with 64 assertions. Read and parse failures,
truncated and absent bundles preserve the previous output and skip Recast. A valid
in-place retry produces the original bytes. No production defect was reproduced
by these cases. Existing transformation and geometry tests already cover the
requested synthetic invariants, so no duplicate cases were added.

The acceptance command fixes a verification weakness: fixture validation previously
relied on preset/cache state to enable native DIG transfers, and CTest could accept
an empty test selection. Both are now explicit. It also makes the non-native build
part of the same unattended run.

An isolated copy of the wrapper was exercised with an injected child-script
failure. It stopped after the first check, propagated the exception, and recorded
both check and overall status as `Failed` with a completion timestamp. The generated
test files stayed under Extractor `out/`; the real acceptance run was unaffected.

After the audit, the user explicitly deferred the built-in minimum-area build-domain
policy. No measured performance need justifies adding this optional filter now;
small islands may provide necessary routes. Custom domain callbacks and route-area
preferences remain available. No known required feature implementation remains in
the currently accepted fixture-tested DIG/Extractor scope. The deferred validation,
integration and resource work below still limits full V2 rollout readiness.

Extractor collision snapshots currently expose environment revision zero. Such
graphs remain explicitly ineligible for generic persistent validation reuse.
Bundle rebuild instead verifies source/import provenance and revalidates fresh
capture. Hashing a bundle or matching a mesh does not grant complete collision
coverage. No stale validation reuse is introduced by the rebuild flow.

Remaining integration and evidence work stays separate:

- Actual DAT archive reading and GUID-only index resolution are not exercised by
  the injected payload tests. These tests use production PARM/TRN/HAVK parsing.
- Ordinary DAT/OBJ collision remains incomplete. Standalone fixture declarations
  describe only the supplied triangle world. Hollow interiors and narrow supported
  platforms can legitimately pass destination-only validation.
- Host loading, world-unit conversion, policy propagation, movement calibration,
  execution, and package/dependency publication remain deferred.
- Large-map latency and memory baselines, strict process-memory limits, and
  whole-pipeline allocation hardening remain deferred. Bundle accounting excludes
  caller-owned data, JSON/codec internals and DIG compiler overhead as documented.

There is no defensible single completion percentage for these unequal items.
Report delivered contracts, this explicit library feature gap, and deferred
integration/evidence separately. Green automated acceptance is not full V2 rollout.
