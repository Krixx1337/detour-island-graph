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
#include <limits>

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
    for (const auto& r : rects) params.bmax[1] = (std::max)(params.bmax[1], float(r.y + 1));
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
    CHECK((**result.compilation.value).offsets() == BuildVector<std::size_t>{0});
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


namespace {
ProductionBuildOptions productionOptions() {
    ProductionBuildOptions o;
    o.limits = {64 * 1024 * 1024, 1000000, 10000, 10000, 10000, 10000, 10000, 20000};
    o.sampleBatchSize = 3;
    o.candidateBatchSize = 5;
    return o;
}
DiscoveryConfig boundedConfig() { auto c = config(); c.maxSamples = 10000; c.maxCandidates = 10000; return c; }
std::string graphBytes(const CompiledGraph& graph) {
    std::ostringstream out(std::ios::binary);
    REQUIRE(GraphSerializer::write(out, graph) == SerializationStatus::Success);
    return out.str();
}
}

TEST_CASE("V2 bounded exhaustive matches reference across batches and directional policies") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}, {8, 0, 10, 2, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    input.identity = {1, 2, 3, 4, 5, 6, 1, 7};
    ValidationOptions validation;
    CompileOptions compilation;
    SUBCASE("geometric unknown") {}
    SUBCASE("directional validation") {
        validation.validator = [](const ValidationRequest& r) {
            return ValidationResult{r.from.island < r.to.island ? ValidationState::Valid : ValidationState::Invalid, 42};
        };
        compilation.policy = CompilePolicy::ValidatedOnly;
    }
    SUBCASE("excluded destination") {
        input.islandPolicy = [](IslandId i, const IslandMetrics&) {
            return IslandDomain{i == 1 ? DomainState::Excluded : DomainState::Included, 17};
        };
    }
    SUBCASE("outbound policy") { validation.outboundPolicy = [](IslandId i) { return i != 1; }; }
    const auto c = boundedConfig();
    const auto reference = buildGraph(input, c, validation, compilation);
    REQUIRE(reference.compilation.value);
    const auto expected = graphBytes(**reference.compilation.value);
    std::size_t work = 0;
    for (std::size_t batch : {1u, 2u, 7u, 1000u}) {
        auto o = productionOptions();
        o.sampleBatchSize = batch;
        o.candidateBatchSize = batch + 1;
        auto built = buildGraphBounded(input, c, o, validation, compilation);
        REQUIRE(built.status == StageStatus::Success);
        REQUIRE(built.graph);
        CHECK(graphBytes(*built.graph) == expected);
        CHECK(built.stats.samples == reference.sampling.stats.samples);
        CHECK(built.stats.sampleDuplicates == reference.sampling.stats.sampleDuplicates);
        CHECK(built.stats.candidatesVisited == reference.discovery.stats.candidatesVisited);
        CHECK(built.stats.exactDuplicates == reference.validation.stats.exactDuplicates);
        CHECK(built.stats.validatorCalls == reference.validation.stats.validatorCalls);
        CHECK(built.budget.exhausted == BudgetResource::None);
        CHECK(built.budget.peakAllocationBytes > 0);
        CHECK(built.budget.peakSampleBatch <= o.sampleBatchSize);
        CHECK(built.budget.peakCandidateBatch <= o.candidateBatchSize);
        if (work) CHECK(built.budget.workUnits == work);
        work = built.budget.workUnits;
    }
}

TEST_CASE("V2 bounded production permits exact resource caps and rejects next operation") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}, {8, 0, 10, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    const auto config = boundedConfig();
    const auto options = productionOptions();
    auto baseline = buildGraphBounded(input, config, options);
    REQUIRE(baseline.graph);
    struct Case { std::size_t BuildLimits::*field; std::size_t used; BudgetResource resource; };
    const Case cases[] = {
        {&BuildLimits::maxAllocationBytes, baseline.budget.peakAllocationBytes, BudgetResource::AllocationBytes},
        {&BuildLimits::maxWorkUnits, baseline.budget.workUnits, BudgetResource::WorkUnits},
        {&BuildLimits::maxTopologyPolygons, baseline.stats.eligiblePolygons, BudgetResource::TopologyPolygons},
        {&BuildLimits::maxBoundaryIntervals, baseline.stats.boundaryIntervals, BudgetResource::BoundaryIntervals},
        {&BuildLimits::maxNearbyRefsPerQuery, baseline.budget.peakNearbyRefs, BudgetResource::NearbyRefsPerQuery},
        {&BuildLimits::maxUniqueCrossings, baseline.budget.uniqueCrossings, BudgetResource::UniqueCrossings},
        {&BuildLimits::maxCompiledCrossings, baseline.stats.compiledCrossings, BudgetResource::CompiledCrossings},
        {&BuildLimits::maxCompiledDirections, baseline.stats.compiledDirections, BudgetResource::CompiledDirections}
    };
    const auto expected = graphBytes(*baseline.graph);
    for (auto item : cases) {
        CAPTURE(int(item.resource));
        REQUIRE(item.used > 1);
        auto exact = options;
        exact.limits.*item.field = item.used;
        auto succeeded = buildGraphBounded(input, config, exact);
        REQUIRE(succeeded.graph);
        CHECK(graphBytes(*succeeded.graph) == expected);
        exact.limits.*item.field = item.used - 1;
        auto failed = buildGraphBounded(input, config, exact);
        CHECK(failed.status == StageStatus::BudgetExceeded);
        CHECK_FALSE(failed.graph);
        CHECK(failed.budget.exhausted == item.resource);
        CHECK(failed.budget.limit == item.used - 1);
        CHECK(failed.budget.attempted > failed.budget.limit);
        CHECK(failed.budget.peakAllocationBytes <= exact.limits.maxAllocationBytes);
    }
    for (bool sampleLimit : {false, true}) {
        auto c = config;
        auto& cap = sampleLimit ? c.maxSamples : c.maxCandidates;
        cap = sampleLimit ? baseline.stats.samples : baseline.stats.candidatesVisited;
        REQUIRE(buildGraphBounded(input, c, options).graph);
        --cap;
        const auto failed = buildGraphBounded(input, c, options);
        CHECK(failed.status == StageStatus::BudgetExceeded);
        CHECK_FALSE(failed.graph);
        CHECK(failed.budget.exhausted == (sampleLimit ? BudgetResource::Samples : BudgetResource::Candidates));
    }
    // Failed rebuild never touches the caller's previous publication.
    CHECK(graphBytes(*baseline.graph) == expected);
}

TEST_CASE("V2 bounded rejected and duplicate candidates still consume global budgets") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    ValidationOptions validation;
    validation.validator = [](const ValidationRequest&) { return ValidationResult{ValidationState::Invalid, 1}; };
    auto o = productionOptions();
    o.sampleBatchSize = o.candidateBatchSize = 1;
    auto good = buildGraphBounded(input, boundedConfig(), o, validation);
    REQUIRE(good.graph);
    CHECK(good.graph->crossings().empty());
    CHECK(good.stats.exactDuplicates > 0);
    CHECK(good.budget.uniqueCrossings > 0);
    CHECK(good.stats.candidatesVisited > good.budget.uniqueCrossings);
    CHECK(good.stats.validatorCalls == 2 * good.budget.uniqueCrossings);
    o.limits.maxUniqueCrossings = good.budget.uniqueCrossings - 1;
    auto failed = buildGraphBounded(input, boundedConfig(), o, validation);
    CHECK(failed.status == StageStatus::BudgetExceeded);
    CHECK_FALSE(failed.graph);
    CHECK(failed.budget.exhausted == BudgetResource::UniqueCrossings);
}

TEST_CASE("V2 bounded seeded frontier matches reference and budgets span islands") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}, {8, 0, 10, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    auto extracted = extractTopology(input);
    REQUIRE(extracted.value);
    SeededBuildOptions seeds;
    seeds.seedIdentity = 31;
    seeds.seeds = {{0, extracted.value->topology.polygons[0].polygon, {1, 0, 1}}};
    ValidationState forward = ValidationState::Valid;
    SUBCASE("valid chain") {}
    SUBCASE("unknown does not expand") { forward = ValidationState::Unknown; }
    SUBCASE("reverse does not expand") { forward = ValidationState::Invalid; }
    ValidationOptions validation;
    validation.validator = [&](const ValidationRequest& r) {
        return ValidationResult{r.from.island < r.to.island ? forward : ValidationState::Valid, 9};
    };
    auto c = boundedConfig();
    auto reference = buildSeededGraph(input, c, seeds, validation);
    REQUIRE(reference.value);
    const auto expected = graphBytes(**reference.value);
    for (std::size_t batch : {1u, 5u, 1000u}) {
        auto o = productionOptions();
        o.sampleBatchSize = o.candidateBatchSize = batch;
        auto built = buildSeededGraphBounded(input, c, seeds, o, validation);
        REQUIRE(built.graph);
        CHECK(graphBytes(*built.graph) == expected);
        CHECK(built.stats.samples == reference.stats.samples);
        CHECK(built.stats.candidatesVisited == reference.stats.candidatesVisited);
        auto capped = c;
        capped.maxSamples = built.stats.samples;
        capped.maxCandidates = built.stats.candidatesVisited;
        REQUIRE(buildSeededGraphBounded(input, capped, seeds, o, validation).graph);
        --capped.maxSamples;
        auto failed = buildSeededGraphBounded(input, capped, seeds, o, validation);
        CHECK(failed.status == StageStatus::BudgetExceeded);
        CHECK_FALSE(failed.graph);
    }
}

TEST_CASE("V2 bounded cancellation callback failures and invalid controls publish nothing") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    auto o = productionOptions();
    auto c = boundedConfig();
    ValidationOptions v;
    CompileOptions compilation;
    auto expected = StageStatus::Canceled;
    SUBCASE("input cancel") { input.canceled = [] { return true; }; }
    SUBCASE("validation cancel") { v.canceled = [] { return true; }; }
    SUBCASE("compilation cancel applies throughout") { compilation.canceled = [] { return true; }; }
    SUBCASE("polygon callback exception") {
        input.polygonFilter = [](dtPolyRef, const dtMeshTile&, const dtPoly&) -> bool { throw std::runtime_error("test"); };
        expected = StageStatus::CallbackFailed;
    }
    SUBCASE("validator bad_alloc remains callback failure") {
        v.validator = [](const ValidationRequest&) -> ValidationResult { throw std::bad_alloc(); };
        expected = StageStatus::CallbackFailed;
    }
    SUBCASE("cancel callback exception") {
        compilation.canceled = []() -> bool { throw std::runtime_error("test"); };
        expected = StageStatus::CallbackFailed;
    }
    SUBCASE("unset limits") { o = {}; expected = StageStatus::InvalidInput; }
    SUBCASE("zero candidate batch") { o.candidateBatchSize = 0; expected = StageStatus::InvalidInput; }
    SUBCASE("uncapped samples forbidden") { c.maxSamples = 0; expected = StageStatus::InvalidInput; }
    auto result = buildGraphBounded(input, c, o, v, compilation);
    CHECK(result.status == expected);
    CHECK_FALSE(result.graph);
}

TEST_CASE("V2 bounded every cancellation checkpoint releases unpublished graph") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    std::size_t calls = 0;
    std::size_t stop = (std::numeric_limits<std::size_t>::max)();
    input.canceled = [&] { return ++calls == stop; };
    auto reference = buildGraphBounded(input, boundedConfig(), productionOptions());
    REQUIRE(reference.graph);
    const auto total = calls;
    const auto bytes = graphBytes(*reference.graph);
    // Includes topology, collector, all compilation loops, and final publication.
    for (stop = 1; stop <= total; ++stop) {
        calls = 0;
        auto result = buildGraphBounded(input, boundedConfig(), productionOptions());
        CHECK(result.status == StageStatus::Canceled);
        CHECK_FALSE(result.graph);
    }
    CHECK(graphBytes(*reference.graph) == bytes);
}

TEST_CASE("V2 bounded callbacks can reenter without inheriting allocation or work budgets") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    auto normal = buildGraphBounded(input, boundedConfig(), productionOptions());
    REQUIRE(normal.graph);
    auto limits = productionOptions();
    limits.limits.maxAllocationBytes = normal.budget.peakAllocationBytes;
    ValidationOptions v;
    v.validator = [&](const ValidationRequest&) {
        auto inner = buildGraphBounded(input, boundedConfig(), productionOptions());
        CHECK(inner.status == StageStatus::Success);
        return ValidationResult{};
    };
    auto nested = buildGraphBounded(input, boundedConfig(), limits, v);
    REQUIRE(nested.graph);
    CHECK(nested.budget.peakAllocationBytes == normal.budget.peakAllocationBytes);
    // Returned storage outlives build scope; another independent build cannot affect it.
    CHECK(nested.graph->crossings().size() == normal.graph->crossings().size());
    CHECK_FALSE(graphBytes(*nested.graph).empty());
}


TEST_CASE("V2 bounded collector rejects dense stacked query before a 65th ref is stored") {
    std::vector<Rect> rects;
    for (unsigned short y = 0; y < 80; ++y) rects.push_back({0, 0, 2, 2, y});
    auto mesh = makeMesh(rects);
    BuildInput input;
    input.navMesh = mesh.get();
    auto c = boundedConfig();
    c.maxClimb = c.maxDrop = 100;
    auto options = productionOptions();
    options.limits.maxNearbyRefsPerQuery = 64;
    auto result = buildGraphBounded(input, c, options);
    CHECK(result.status == StageStatus::BudgetExceeded);
    CHECK_FALSE(result.graph);
    CHECK(result.budget.exhausted == BudgetResource::NearbyRefsPerQuery);
    CHECK(result.budget.peakNearbyRefs == 64);
    CHECK(result.budget.attempted == 65);
    CHECK(result.stats.discoveryQueries == 1);
    CHECK(result.stats.validatorCalls == 0);
}

TEST_CASE("V2 bounded empty domain is complete and seeded input remains checked") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    SUBCASE("excluded all") {
        input.islandPolicy = [](IslandId, const IslandMetrics&) { return IslandDomain{DomainState::Excluded, 1}; };
        auto r = buildGraphBounded(input, boundedConfig(), productionOptions());
        REQUIRE(r.graph);
        CHECK(r.graph->coverage().complete);
        CHECK(r.stats.excludedIslands == 2);
        CHECK(r.stats.samples == 0);
        CHECK(r.graph->crossings().empty());
    }
    SUBCASE("no eligible polygons") {
        input.polygonFilter = [](dtPolyRef, const dtMeshTile&, const dtPoly&) { return false; };
        auto r = buildGraphBounded(input, boundedConfig(), productionOptions());
        REQUIRE(r.graph);
        CHECK(r.graph->offsets().size() == 1);
        CHECK(r.stats.samples == 0);
    }
    SUBCASE("bad required seed") {
        SeededBuildOptions seeds;
        seeds.seeds = {{0, 0, {1, 0, 1}}};
        ValidationOptions validation;
        validation.validator = [](const ValidationRequest&) { return ValidationResult{ValidationState::Valid, 0}; };
        auto r = buildSeededGraphBounded(input, boundedConfig(), seeds, productionOptions(), validation);
        CHECK(r.status == StageStatus::InvalidInput);
        CHECK_FALSE(r.graph);
    }
}


TEST_CASE("V2 bounded invalid geometry settings are rejected before allocations and callbacks") {
    auto mesh = makeMesh({{0, 0, 2, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    int calls = 0;
    input.canceled = [&] { ++calls; return false; };
    auto c = boundedConfig();
    c.sampleSpacing = 0;
    auto o = productionOptions();
    o.limits.maxAllocationBytes = 1;
    auto r = buildGraphBounded(input, c, o);
    CHECK(r.status == StageStatus::InvalidInput);
    CHECK_FALSE(r.graph);
    CHECK(r.budget.peakAllocationBytes == 0);
    CHECK(calls == 0);
}
