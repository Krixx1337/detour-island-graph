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

These are source geometry fixtures, not baked `.nav` files or established jump
validation expectations. No scaling or coordinate conversion has been applied.
Exports omit mapmodels and materials such as water, lava, and glass.

When adding baked navmeshes, record the source geometry hash, baker revision,
complete bake settings, coordinate transform and scale, and movement policy.
Preserve matching collision geometry for swept jump validation. Navmesh-only
tests can exercise topology, discovery, determinism, and budgets, but cannot
establish collision-free jumps.
