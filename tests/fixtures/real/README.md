# Real map fixture sources

Small starter set copied unchanged from the user-provided `game-maps-obj-master`
download on 2026-09-08. Upstream: https://github.com/Calinou/game-maps-obj.
The downloaded snapshot's commit is unknown. `UPSTREAM_README.md` preserves its
export details, license table, and geometry limitations.

- `pandora/pandora.obj`: Sauerbraten indoor baseline for rooms and corridors.
  Upstream identifies CC BY 4.0; supplied `pandora.txt` is preserved.
- `steelribs/steelribs.obj`: Tesseract floating geometry for disconnected regions
  and jump candidate discovery. Upstream identifies CC BY-SA 3.0. This download
  has no map-specific `steelribs.txt`; the upstream README is the supplied license
  record for this map.

Matching MTL files are included because the OBJ files reference them. Textures
are not included and are not needed for triangle-based collision or nav baking.
These third-party assets retain their upstream licenses.

Each map includes a baked `.nav` and sanitized `.job.json`
from Gw2CollisionExtractor 0.1.0 on 2026-09-08. These files were
copied from the bake session, then sanitized to remove machine-specific paths.
Jobs use filenames relative to the working directory; run from the respective
map folder or resolve those filenames to absolute paths before replaying.
Session NDJSON logs are omitted; relevant provenance is recorded below.
The exact baker commit
is not recorded in the supplied provenance.

Source OBJ files remain unchanged. Baking applies `(x,y,z) -> (x,z,-y)` with
scale `0.05`. Pandora uses agent radius `1.1`; steelribs uses `0.3`.
Complete settings are in the accompanying jobs.
Both reports indicate success and two walkable tiles. Library tests now load
both tiles and verify topology and geometric candidates. Collision-free jump
expectations remain unverified.
Exports omit mapmodels and materials such as water, lava, and glass.

Bake diagnostics recorded these source hashes using the extractor's hash scheme:

| Fixture | Source hash | Source bytes | Accepted triangles | Skipped degenerate faces |
| --- | --- | ---: | ---: | ---: |
| Pandora | `09e872c3bb78586c` | 3562248 | 60694 | 116 |
| Steelribs | `12712fcfc2bd38b7` | 2725793 | 45846 | 58 |

Both bakes reported exit code 0, no bad-index or non-finite skips, and an
`obj_source` warning at the `havok` stage. These hashes are not SHA-256.

When adding baked navmeshes, record the source geometry hash, baker revision,
complete bake settings, coordinate transform and scale, and movement policy.
Preserve matching collision geometry for swept jump validation. Navmesh-only
tests can exercise topology, discovery, determinism, and budgets, but cannot
establish collision-free jumps.

## Regression baseline

Discovery uses spacing 0.5, horizontal gap 2, climb 1, and drop 3 in baked
navmesh units. These are test settings, not a verified GW2 movement policy.

| Fixture | Polygons | Islands | Boundary intervals | Unique candidates |
| --- | ---: | ---: | ---: | ---: |
| Pandora | 134 | 25 | 300 | 99 |
| Steelribs | 69 | 15 | 145 | 236 |

Reference and bounded exhaustive graph bytes match for batch sizes 1 and 64.
All unvalidated directions remain Unknown. A validator returning Unknown
produces no executable directions under ValidatedOnly policy; that policy
requires a supplied validator.

MSVC Debug observations on 2026-09-08: Pandora about 8-9 ms and 66,360 peak
accounted bytes; Steelribs about 4-5 ms and 88,272 bytes. These are single-machine
observations, not timing assertions or process-memory measurements. Fixed test
budgets allow 1 MiB and 100,000 work units, with explicit count caps.

The test-only loader checks MSET envelopes and native tile section sizes. It
assumes trusted fixture polygon data and the compatible native Detour ABI;
it is not a general untrusted-file parser. Tests also cover malformed envelopes,
allocation/work exhaustion, and cancellation without graph publication.
