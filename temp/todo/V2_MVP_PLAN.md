# DetourIslandGraph v2 MVP

## Summary

Implementation status: see [V2_PROGRESS.md](../../V2_PROGRESS.md). Foundation
contracts, directional validation, and graph compilation are implemented;
mesh extraction/discovery, routing, persistence, and host migration remain pending.

Rebuild v2 around explicit boundary sampling, independently validated traversal directions, and separate graph compilation.

Keep generation as post-processing after Recast baking. Keep symmetric-first storage and existing host working through a full API/cache migration.

Prioritize coverage over build speed. Never silently discard samples to satisfy island quotas.

## 1. New build pipeline and public contracts

Separate build into three callable stages, plus one convenience wrapper:

```text
Extract topology and sample boundaries
                  ↓
Discover and validate crossing candidates
                  ↓
Compile immutable graph
```

- `BuildInput` contains frozen navmesh, build identity, polygon filter, and cancellation callback.
- `DiscoveryConfig` contains explicit boundary spacing, horizontal/up/down limits, outbound policy, and resource limits.
- `CrossingArtifact` contains island topology, endpoint polygon anchors, canonical crossing geometry, direction results, and build provenance.
- `CompileOptions` selects geometric-only or validated-only traversal.
- Compiled graph contains compact crossing storage, adjacency, polygon lookup, and precomputed portal offsets.
- Keep all coordinates in navmesh units. Host converts settings at its existing adapter boundary.
- Remain C++17. No new third-party dependencies or Recast-baker dependency.

Artifacts reusable in memory for graph compilation. MVP persists compiled graphs only; separate on-disk artifact caching deferred.

## 2. Discovery and validation

### Boundary sampling

- Flood-fill only eligible native ground connectivity. Generated crossings never merge islands.
- Extract exposed edge intervals. For external portals, subtract union of linked intervals leading to eligible neighboring polygons.
- Treat supplied navmesh snapshot as complete. Streaming/unloaded-neighbor semantics deferred.
- Sample each interval at both endpoints and evenly spaced interior positions. Maximum spacing must not exceed configured value.
- Require positive, explicit `sampleSpacing`. Host starts at its existing 4-metre boundary setting, converted into navmesh units.
- Deduplicate identical sample positions within the same island, preserving deterministic ownership.
- Query nearby eligible polygons with existing Detour spatial queries; project candidate endpoints and apply independent horizontal/up/down limits.
- Discover crossings between different islands only.

Remove v1 mass quotas, small-island suppression, representative reduction, pair-scan suppression, short-gap recovery, approximate candidate voxel merging, and local/global/spanner pruning.

Retain mass scores solely for existing host route preferences and diagnostics. They must not suppress geometry.

MVP merges exact duplicate endpoint pairs only. No approximate pruning or span merging.

### Directional validation

Each canonical crossing stores geometry once, with separate AB and BA records:

- Geometric eligibility and outbound-policy permission.
- `Valid`, `Invalid`, or `Unknown` validation result.
- Validator reason code.

One movement profile per build. Profile and validator semantic IDs belong in build identity; multiple action variants deferred.

Validator receives ordered endpoints, polygon anchors, and movement-profile context. Caller supplies frozen collision/environment access through its implementation.

- Invoke validator independently for every geometrically eligible, policy-allowed direction.
- No validator means `Unknown`.
- Never infer reverse validity from equal climb/drop limits.
- Validate before any non-exact candidate elimination.
- Geometric-only compilation permits eligible `Valid` and `Unknown` directions, never `Invalid`.
- Validated-only compilation permits only `Valid`. Missing validator is configuration error.
- Omit crossings with no usable compiled directions.
- Callback failure or cancellation aborts build; never publish partial graph.

Host explicitly selects geometric-only mode for current gap behavior. No built-in parabola, collision backend, or movement execution in MVP.

### Resource behavior

- Explicit optional sample and candidate caps; zero means uncapped.
- Reaching a cap returns `BudgetExceeded` with counters and no publishable graph.
- Cancellation checked through sampling, queries, validation, compilation, and diagnostics.
- Do not shrink sampling density automatically.

## 3. Routing, persistence, and host migration

### Routing

Keep portal-based routing between islands.

- Add transfer-cost callback receiving island and anchored endpoints.
- Default transfer cost remains Euclidean and is explicitly reported as estimated.
- Preserve custom crossing cost/filter callbacks.
- Use geometric A* only with built-in Euclidean costs. Any custom cost provider uses Dijkstra in MVP; remove custom heuristic callback.
- Accept finite, nonnegative costs; represent blocked traversal separately from numeric cost.
- Stop search when best remaining search bound cannot improve completed route.
- Reuse precomputed portal offsets and caller-owned scratch storage. No mutable search state inside shared graph.
- Same-island queries retain `SameIsland` result; no same-island shortcut search.
- No actual Detour transfer-cost integration or cross-query transfer cache in MVP.

### Cache and API break

- Set public package version to `2.0.0`.
- Replace old public configuration and graph contracts rather than retaining compatibility wrappers.
- Introduce new native serialization version and host cache version. Reject older graphs and rebuild.
- Cache identity covers navmesh fingerprint, coordinate scale, polygon policy, discovery settings, movement profile, validator identity, and compilation policy.
- Custom semantic callbacks without stable caller-provided identity disable persistent reuse.
- Validate decoded counts, references, finite geometry, directional states, and adjacency before accepting graph.
- Preserve host protected-cache wrapping, atomic replacement, cancellation, and immutable publication.

### Host integration

Update adapter, build settings, route callbacks, cache handling, reports, and UI controls together.

- Remove controls and counters for deleted heuristics.
- Preserve current route preferences and movement ownership.
- Keep builds on existing maintenance lane; no new scheduler or background service.
- Retain existing landing adjustment and execution rules. Validation results must not claim those execution paths are physical jumps.
- Old configuration keys have no migration requirement.

## 4. Diagnostics and acceptance tests

Basic build report includes effective spacing/capabilities, sample count, projection count, exact duplicates, directional validation outcomes, graph size, reachability, disconnected components, and per-stage timing.

Keep nearest-representative coverage diagnostics opt-in. Basic reporting must not trigger quadratic nearest-neighbor analysis.

Add focused tests for:

- Long-edge opportunities near endpoints.
- Partial external portals and multiple linked intervals.
- Large islands receiving configured spacing without quota thinning.
- Unequal climb/drop limits and validator rejecting only one direction.
- Unknown validation under geometric-only versus validated-only compilation.
- Exact duplicates preserving canonical geometry and directional results.
- Small islands remaining discoverable.
- Budget exhaustion and cancellation producing no publishable graph.
- Routing callback costs, blocked transfers, Dijkstra ordering, and estimated-cost labeling.
- New serialization round-trip, corrupt input rejection, identity mismatch, and old-cache rejection.
- Scale-adjusted configurations, translation, tile order, and equivalent retessellation using geometric tolerances rather than identical sample IDs.

Verification sequence:

1. Build library test target and run focused tests.
2. Run host dependency checks and available targeted compilation checks.
3. Run Queensdale comparison with unchanged traversal capabilities. Record sample coverage, graph size, build time, peak memory, recovered exits, and disconnected components.
4. Exercise real host routes, cancellation, graph rebuild, and stale-cache rejection.

No fixed one-second build target. Coverage-first choice permits slower builds, but measurements must expose cost. Passing requires no silent sampling loss and no structural/directional violations; 100% map connectivity is not a requirement.

Full application build and in-game checks remain explicit handoff checks when unavailable or not authorized.

## 5. Implementation order and exclusions

Deliver in this order:

1. New data contracts and focused contract tests.
2. Exposed-interval extraction and explicit sampling.
3. Directional validator integration and exact graph compilation.
   Before full host migration, benchmark the dense exact-only pipeline on
   Queensdale for graph size, build time, and peak memory. Revisit simplification
   only if measurements justify changing MVP scope. Measure query latency once
   the V2 router is available.
4. Routing contract and serialization replacement.
5. Host migration, diagnostics, and Queensdale validation.

Out of v2 MVP:

- Built-in physical jump solver or collision engine.
- Multiple movement profiles per artifact.
- Crossing spans and approximate simplification.
- Same-island shortcuts and regional routing.
- Incremental tile rebuilds or artifact disk caching.
- Worker parallelization and new library dependencies.

These exclusions keep MVP focused: predictable discovery, honest validity, simpler configuration, and working host integration.
