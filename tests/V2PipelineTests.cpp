#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>
#include <DetourNavMeshBuilder.h>

#include <array>
#include <memory>
#include <vector>

namespace {
using namespace detour_island_graph::v2;
constexpr unsigned short border = 0xffff;

struct Rect {
    unsigned short x0, z0, x1, z1, y = 0;
    std::array<unsigned short, 4> neighbors{border, border, border, border};
};

struct MeshDeleter { void operator()(dtNavMesh* mesh) const { dtFreeNavMesh(mesh); } };
using Mesh = std::unique_ptr<dtNavMesh, MeshDeleter>;

Mesh makeMesh(const std::vector<Rect>& rects) {
    std::vector<unsigned short> verts, polys;
    std::vector<unsigned short> flags(rects.size(), 1);
    std::vector<unsigned char> areas(rects.size(), 0);
    for (const auto& r : rects) {
        const auto base = static_cast<unsigned short>(verts.size() / 3);
        verts.insert(verts.end(), {r.x0, r.y, r.z0, r.x1, r.y, r.z0,
            r.x1, r.y, r.z1, r.x0, r.y, r.z1});
        for (unsigned short i = 0; i < 4; ++i) polys.push_back(base + i);
        polys.insert(polys.end(), r.neighbors.begin(), r.neighbors.end());
    }
    dtNavMeshCreateParams params{};
    params.verts = verts.data();
    params.vertCount = int(verts.size() / 3);
    params.polys = polys.data();
    params.polyFlags = flags.data();
    params.polyAreas = areas.data();
    params.polyCount = int(rects.size());
    params.nvp = 4;
    params.bmin[0] = 0;
    params.bmin[1] = 0;
    params.bmin[2] = 0;
    params.bmax[0] = 10;
    params.bmax[1] = 5;
    params.bmax[2] = 10;
    params.walkableHeight = 2;
    params.walkableRadius = 0.5f;
    params.walkableClimb = 0.5f;
    params.cs = 1;
    params.ch = 1;
    params.buildBvTree = true;
    int size = 0;
    unsigned char* data = nullptr;
    REQUIRE(dtCreateNavMeshData(&params, &data, &size));
    Mesh mesh(dtAllocNavMesh());
    REQUIRE(mesh);
    REQUIRE(dtStatusSucceed(mesh->init(data, size, DT_TILE_FREE_DATA)));
    return mesh;
}

DiscoveryConfig config() { return {2, 3, 2, 4, 0, 0}; }

PipelineResult build(const Mesh& mesh, DiscoveryConfig settings = config(),
    ValidationOptions validation = {}, CompileOptions compilation = {}) {
    BuildInput input;
    input.navMesh = mesh.get();
    return buildGraph(input, settings, validation, compilation);
}
} // namespace

TEST_CASE("V2 pipeline builds a usable graph with preserved stage artifacts") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    const auto result = build(mesh);
    CHECK(result.status == StageStatus::Success);
    REQUIRE(result.sampling.value);
    REQUIRE(result.discovery.value);
    REQUIRE(result.validation.value);
    REQUIRE(result.compilation.value);
    CHECK(result.sampling.status == StageStatus::Success);
    CHECK(result.discovery.status == StageStatus::Success);
    CHECK(result.validation.status == StageStatus::Success);
    CHECK(result.compilation.status == StageStatus::Success);
    CHECK(result.sampling.value->topology.islandCount == 2);
    CHECK_FALSE(result.discovery.value->empty());
    CHECK(result.validation.stats.candidatesVisited == result.discovery.value->size());
    const auto& graph = **result.compilation.value;
    CHECK_FALSE(graph.crossings().empty());
    CHECK(graph.offsets().size() == 3);
    CHECK(result.timings.totalMs >= 0);
    CHECK(result.timings.samplingMs >= 0);
    CHECK(result.timings.discoveryMs >= 0);
    CHECK(result.timings.validationMs >= 0);
    CHECK(result.timings.compilationMs >= 0);

    // Preserved artifacts support policy recompilation without new validation.
    const auto recompiled = compileGraph(*result.validation.value);
    REQUIRE(recompiled.value);
    CHECK((**recompiled.value).crossings().size() == graph.crossings().size());
}

TEST_CASE("V2 pipeline empty input succeeds with an empty graph") {
    auto mesh = makeMesh({{0, 0, 2, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    input.polygonFilter = [](dtPolyRef, const dtMeshTile&, const dtPoly&) { return false; };
    const auto result = buildGraph(input, config());
    CHECK(result.status == StageStatus::Success);
    REQUIRE(result.compilation.value);
    CHECK((**result.compilation.value).crossings().empty());
    CHECK((**result.compilation.value).offsets() == std::vector<std::size_t>{0});
}

TEST_CASE("V2 pipeline propagates stage failures with no graph") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});

    auto settings = config();
    settings.maxSamples = 1;
    const auto sampled = build(mesh, settings);
    CHECK(sampled.status == StageStatus::BudgetExceeded);
    CHECK_FALSE(sampled.compilation.value);
    CHECK(sampled.sampling.status == StageStatus::BudgetExceeded);

    settings = config();
    settings.maxCandidates = 1;
    const auto discovered = build(mesh, settings);
    CHECK(discovered.status == StageStatus::BudgetExceeded);
    CHECK_FALSE(discovered.compilation.value);

    const auto strict = build(mesh, config(), {}, {CompilePolicy::ValidatedOnly, {}});
    CHECK(strict.status == StageStatus::InvalidInput);
    CHECK_FALSE(strict.compilation.value);

    BuildInput missing;
    const auto invalid = buildGraph(missing, config());
    CHECK(invalid.status == StageStatus::InvalidInput);
    CHECK_FALSE(invalid.compilation.value);

    BuildInput canceled;
    canceled.navMesh = mesh.get();
    canceled.canceled = [] { return true; };
    const auto aborted = buildGraph(canceled, config());
    CHECK(aborted.status == StageStatus::Canceled);
    CHECK_FALSE(aborted.compilation.value);
}

TEST_CASE("V2 pipeline honors supplied validators end to end") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    ValidationOptions validation;
    validation.validator = [](const ValidationRequest&) {
        return ValidationResult{ValidationState::Valid, 9};
    };
    const auto result = build(mesh, config(), validation, {CompilePolicy::ValidatedOnly, {}});
    CHECK(result.status == StageStatus::Success);
    REQUIRE(result.compilation.value);
    CHECK_FALSE((**result.compilation.value).crossings().empty());
    CHECK(result.validation.stats.validDirections > 0);
}
