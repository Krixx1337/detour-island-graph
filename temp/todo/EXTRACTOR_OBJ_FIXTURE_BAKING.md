# Bake real OBJ fixtures through Gw2CollisionExtractor

## Goal

Use the current Gw2CollisionExtractor baking pipeline to produce reproducible
navmesh fixtures for DetourIslandGraph V2. GW2NavMeshBuilder is outdated and
must not become the fixture baker.

## Scope

- Inspect Gw2CollisionExtractor's existing input support. If needed, add an OBJ
  input mode that feeds indexed triangles into the existing Recast baker.
- Reuse the current bake settings and navmesh output format. Keep GW2 collision
  extraction and OBJ loading as input adapters to the shared baking pipeline.
- Keep the extractor a stateless worker. The caller supplies input paths,
  output paths, transforms, and complete bake settings through its job contract.
- Bake the starter fixtures in `tests/fixtures/real`: `pandora` and `steelribs`.
- Preserve each source OBJ alongside its baked navmesh. Record source hash,
  extractor revision, bake settings, coordinate transform, and scale so the
  navmesh and collision geometry can be reproduced and matched.

## Validation

- Confirm generated files load through the supported Detour navmesh loader.
- Use the baked fixtures to check V2 topology, candidate discovery,
  deterministic graph output, and build budgets.
- Keep collision and navmesh in the same coordinate system. OBJ geometry is
  required for later jump-clearance validation; navmesh-only tests cannot prove
  that a jump clears walls or ceilings.

## Follow-on work

Integrate automatic jump generation into the extractor after final dtNavMesh
assembly, while matching collision geometry remains resident. Use the bounded
exhaustive V2 builder by default, validate jumps against collision, and export
matched navmesh and graph data with compatible polygon references and identity.
Seeded generation remains optional.

This task does not require completing jump validation, host integration, or
adding material/texture rendering. It establishes fixture baking through the
current production baker.
