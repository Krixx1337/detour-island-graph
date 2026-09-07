#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>
#include <detour_island_graph/v2/Serialization.h>
#include <detour_island_graph/v2/Routing.h>
#include <DetourNavMeshBuilder.h>

#include <array>
#include <memory>
#include <vector>
#include <sstream>
#include <stdexcept>

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

TEST_CASE("V2 seeded frontier follows only validated forward reach and persists coverage") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}, {8, 0, 10, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    input.identity.mesh = 1;
    input.identity.movementProfile = 2;
    input.identity.validator = 3;
    input.identity.environment = 4;
    auto topology = extractTopology(input);
    REQUIRE(topology.value);
    SeededBuildOptions seeds;
    seeds.seedIdentity = 5;
    seeds.seeds = {{0, topology.value->topology.polygons[0].polygon, {1, 0, 1}}};
    ValidationState forward = ValidationState::Valid;
    std::size_t included = 3;
    SUBCASE("forward chain") {}
    SUBCASE("reverse only") { forward = ValidationState::Invalid; included = 1; }
    SUBCASE("unknown forward") { forward = ValidationState::Unknown; included = 1; }
    ValidationOptions validation;
    validation.validator = [&](const ValidationRequest& request) {
        return ValidationResult{request.from.island < request.to.island ? forward : ValidationState::Valid, 0};
    };
    const auto result = buildSeededGraph(input, config(), seeds, validation);
    REQUIRE(result.value);
    const auto& graph = **result.value;
    CHECK(graph.coverage().seeded);
    CHECK(graph.coverage().complete);
    CHECK(graph.coverage().seedIdentity == 5);
    CHECK(graph.persistentReuseEligible());
    CHECK(graph.policy() == CompilePolicy::ValidatedOnly);
    CHECK(result.stats.includedIslands == included);
    CHECK(result.stats.samples == included * 4);
    CHECK(graph.includes(2) == (included == 3));
    CHECK(findRoute(graph, 0, 2, {1, 0, 1}, {9, 0, 1}).status ==
        (included == 3 ? RouteStatus::Success : RouteStatus::OutOfDomain));
    std::ostringstream bytes;
    REQUIRE(GraphSerializer::write(bytes, graph) == SerializationStatus::Success);
    std::istringstream stream(bytes.str());
    auto decoded = GraphSerializer::read(stream);
    REQUIRE(decoded.graph);
    CHECK(decoded.graph->coverage().seeded);
    CHECK(decoded.graph->coverage().seeds.size() == 1);
    CHECK(decoded.graph->coverage().seedIdentity == 5);
    CHECK(decoded.graph->includes(2) == graph.includes(2));
    auto corrupt = bytes.str();
    // One 24-byte seed follows the 14-byte coverage header.
    corrupt[corrupt.size() - 24 - 13] = 0;
    std::istringstream incomplete(corrupt);
    CHECK(GraphSerializer::read(incomplete).status == SerializationStatus::MalformedData);
}

TEST_CASE("V2 seeded builds reject bad seeds budgets and callbacks without publication") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}, {8, 0, 10, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    auto topology = extractTopology(input);
    REQUIRE(topology.value);
    SeededBuildOptions seeds;
    seeds.seeds = {{0, topology.value->topology.polygons[0].polygon, {1, 0, 1}}};
    ValidationOptions validation;
    validation.validator = [](const ValidationRequest&) { return ValidationResult{ValidationState::Valid, 0}; };
    auto settings = config();
    auto expected = StageStatus::InvalidInput;
    SUBCASE("no validator") { validation.validator = {}; }
    SUBCASE("no seeds") { seeds.seeds.clear(); }
    SUBCASE("wrong polygon") { seeds.seeds[0].polygon = 0; }
    SUBCASE("wrong island") { seeds.seeds[0].island = 1; }
    SUBCASE("wrong vertical layer") { seeds.seeds[0].position.y = 3; seeds.projectionTolerance = 1; }
    SUBCASE("outside polygon even with wide tolerance") {
        seeds.seeds[0].position.x = 3; seeds.projectionTolerance = 10;
    }
    SUBCASE("excluded seed") {
        input.islandPolicy = [](IslandId, const IslandMetrics&) { return IslandDomain{DomainState::Excluded, 1}; };
    }
    SUBCASE("global sample cap") { settings.maxSamples = 8; expected = StageStatus::BudgetExceeded; }
    SUBCASE("global candidate cap") { settings.maxCandidates = 5; expected = StageStatus::BudgetExceeded; }
    SUBCASE("validator throws") {
        expected = StageStatus::CallbackFailed;
        validation.validator = [](const ValidationRequest&) -> ValidationResult { throw std::runtime_error("failure"); };
    }
    SUBCASE("cancel from validator") {
        expected = StageStatus::Canceled;
        validation.validator = [&](const ValidationRequest&) {
            input.canceled = [] { return true; };
            return ValidationResult{ValidationState::Valid, 0};
        };
    }
    const auto result = buildSeededGraph(input, settings, seeds, validation);
    CHECK(result.status == expected);
    CHECK_FALSE(result.value);
}

TEST_CASE("V2 seeded reverse expansion is deterministic and respects exact caps and outbound policy") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}, {8, 0, 10, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    const auto topology = extractTopology(input);
    REQUIRE(topology.value);
    SeededBuildOptions seeds;
    seeds.seeds = {{2, topology.value->topology.polygons[2].polygon, {9, 0, 1}}};
    ValidationOptions validation;
    validation.validator = [](const ValidationRequest& request) {
        return ValidationResult{request.from.island > request.to.island ? ValidationState::Valid : ValidationState::Invalid, 0};
    };
    const auto original = buildSeededGraph(input, config(), seeds, validation);
    REQUIRE(original.value);
    CHECK((**original.value).includes(0));
    CHECK(original.stats.includedIslands == 3);
    seeds.seeds.push_back(seeds.seeds.front());
    auto exact = config();
    exact.maxSamples = original.stats.samples;
    exact.maxCandidates = original.stats.candidatesVisited;
    const auto repeated = buildSeededGraph(input, exact, seeds, validation);
    REQUIRE(repeated.value);
    CHECK((**repeated.value).coverage().seeds.size() == 1);
    CHECK(repeated.stats.validatorCalls == original.stats.validatorCalls);
    CHECK(repeated.stats.samples == original.stats.samples);
    CHECK((**repeated.value).crossings().size() == (**original.value).crossings().size());
    validation.outboundPolicy = [](IslandId island) { return island != 1; };
    const auto blocked = buildSeededGraph(input, config(), seeds, validation);
    REQUIRE(blocked.value);
    CHECK((**blocked.value).includes(1));
    CHECK_FALSE((**blocked.value).includes(0));
    CHECK(blocked.stats.includedIslands == 2);
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
