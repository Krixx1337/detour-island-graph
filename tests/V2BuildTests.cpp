#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace {
using namespace detour_island_graph::v2;

TopologyArtifact topology() {
    TopologyArtifact result;
    result.identity.mesh = 1;
    result.identity.movementProfile = 2;
    result.islandCount = 2;
    result.polygons = {{11, 0}, {22, 1}};
    return result;
}

DiscoveryConfig config() { return {1, 5, 3, 6, 0, 0}; }
CrossingCandidate candidate() { return {{0, 11, {0, 0, 0}}, {1, 22, {3, 2, 0}}}; }

TEST_CASE("v2 canonical geometry retains independently validated directions") {
    auto pair = candidate();
    std::swap(pair.a, pair.b);
    ValidationOptions options;
    options.validator = [](const ValidationRequest& request) {
        CHECK(request.identity.movementProfile == 2);
        CHECK(request.from.polygon == (request.from.island == 0 ? 11 : 22));
        return ValidationResult{request.from.island == 0 ? ValidationState::Invalid : ValidationState::Valid, 71};
    };
    auto limits = config();
    limits.maxDrop = limits.maxClimb;
    const auto artifact = validateCrossings(topology(), limits, {pair, candidate()}, options);
    REQUIRE(artifact.status == StageStatus::Success);
    REQUIRE(artifact.value);
    CHECK(artifact.stats.validatorCalls == 2);
    CHECK(artifact.stats.exactDuplicates == 1);
    REQUIRE(artifact.value->crossings.size() == 1);
    CHECK(artifact.value->crossings[0].a.island == 0);
    CHECK(artifact.value->crossings[0].ab.validation.reason == 71);
    for (auto policy : {CompilePolicy::GeometricOnly, CompilePolicy::ValidatedOnly}) {
        const auto compiled = compileGraph(*artifact.value, {policy, {}});
        REQUIRE(compiled.value);
        const auto& graph = **compiled.value;
        REQUIRE(graph.crossings().size() == 1);
        CHECK_FALSE(graph.crossings()[0].traversableAB);
        CHECK(graph.crossings()[0].traversableBA);
        CHECK(graph.offsets() == std::vector<std::size_t>{0, 0, 1});
        REQUIRE(graph.traversals().size() == 1);
        CHECK(graph.traversals()[0].reverse);
        CHECK(graph.polygonIslands().at(22) == 1);
    }
}

TEST_CASE("v2 unknown validation is explicit and compilation never calls validator") {
    auto result = validateCrossings(topology(), config(), {candidate()});
    REQUIRE(result.value);
    CHECK(result.stats.unknownDirections == 2);
    CHECK(compileGraph(*result.value).stats.compiledDirections == 2);
    auto strict = compileGraph(*result.value, {CompilePolicy::ValidatedOnly, {}});
    CHECK(strict.status == StageStatus::InvalidInput);
    CHECK_FALSE(strict.value);

    int calls = 0;
    ValidationOptions options;
    options.validator = [&](const ValidationRequest&) { ++calls; return ValidationResult{}; };
    result = validateCrossings(topology(), config(), {candidate()}, options);
    REQUIRE(result.value);
    strict = compileGraph(*result.value, {CompilePolicy::ValidatedOnly, {}});
    REQUIRE(strict.value);
    CHECK((**strict.value).crossings().empty());
    CHECK(compileGraph(*result.value).stats.compiledDirections == 2);
    CHECK(calls == 2);
}

TEST_CASE("v2 climb drop and outbound limits gate validation independently") {
    auto limits = config();
    limits.maxClimb = 1;
    ValidationOptions options;
    options.validator = [](const ValidationRequest& request) {
        CHECK(request.from.island == 1);
        return ValidationResult{ValidationState::Valid, 0};
    };
    auto result = validateCrossings(topology(), limits, {candidate()}, options);
    REQUIRE(result.value);
    CHECK(result.stats.validatorCalls == 1);
    CHECK_FALSE(result.value->crossings[0].ab.geometricallyEligible);
    CHECK(compileGraph(*result.value).stats.compiledDirections == 1);

    options.outboundPolicy = [](IslandId island) { return island == 0; };
    result = validateCrossings(topology(), limits, {candidate()}, options);
    REQUIRE(result.value);
    CHECK(result.stats.validatorCalls == 0);
    CHECK(compileGraph(*result.value).stats.compiledCrossings == 0);
}

TEST_CASE("v2 caps cancellation and callback failure publish nothing") {
    auto limits = config();
    limits.maxCandidates = 1;
    auto result = validateCrossings(topology(), limits, {candidate(), candidate()});
    CHECK(result.status == StageStatus::BudgetExceeded);
    CHECK_FALSE(result.value);
    CHECK(validateCrossings(topology(), limits, {candidate()}).status == StageStatus::Success);
    ValidationOptions options;
    bool cancel = false;
    options.canceled = [&] { return cancel; };
    options.validator = [&](const ValidationRequest&) {
        cancel = true;
        return ValidationResult{ValidationState::Valid, 0};
    };
    result = validateCrossings(topology(), config(), {candidate()}, options);
    CHECK(result.status == StageStatus::Canceled);
    CHECK_FALSE(result.value);
    options = {};
    options.validator = [](const ValidationRequest&) -> ValidationResult { throw std::runtime_error("test"); };
    result = validateCrossings(topology(), config(), {candidate()}, options);
    CHECK(result.status == StageStatus::CallbackFailed);
    CHECK_FALSE(result.value);
    const auto good = validateCrossings(topology(), config(), {candidate()});
    REQUIRE(good.value);
    int checks = 0;
    const auto compiled = compileGraph(*good.value, {CompilePolicy::GeometricOnly, [&] { return ++checks == 8; }});
    CHECK(compiled.status == StageStatus::Canceled);
    CHECK_FALSE(compiled.value);
}

TEST_CASE("v2 rejects malformed topology geometry config and validation records") {
    auto mesh = topology();
    auto pair = candidate();
    auto limits = config();
    SUBCASE("duplicate polygon") { mesh.polygons.push_back(mesh.polygons.front()); }
    SUBCASE("empty island") { mesh.islandCount = 3; }
    SUBCASE("wrong anchor owner") { pair.a.polygon = 22; }
    SUBCASE("same island") { pair.b = pair.a; }
    SUBCASE("NaN geometry") { pair.a.position.x = (std::numeric_limits<float>::quiet_NaN)(); }
    SUBCASE("infinite limit") { limits.maxDrop = (std::numeric_limits<float>::infinity)(); }
    SUBCASE("missing spacing") { limits.sampleSpacing = 0; }
    SUBCASE("negative reach") { limits.maxHorizontalGap = -1; }
    SUBCASE("invalid scale") { mesh.identity.unitsPerMeter = 0; }
    const auto result = validateCrossings(mesh, limits, {pair});
    CHECK(result.status == StageStatus::InvalidInput);
    CHECK_FALSE(result.value);
}

TEST_CASE("v2 compiler rejects forged permissions and conflicting exact duplicates") {
    auto result = validateCrossings(topology(), config(), {candidate()});
    REQUIRE(result.value);
    auto& artifact = *result.value;
    SUBCASE("conflicting duplicate") {
        artifact.validatorSupplied = true;
        artifact.crossings.push_back(artifact.crossings.front());
        artifact.crossings.back().ba.validation.state = ValidationState::Valid;
    }
    SUBCASE("unsupported validity") { artifact.crossings[0].ab.validation.state = ValidationState::Valid; }
    SUBCASE("invalid enum") { artifact.crossings[0].ab.validation.state = static_cast<ValidationState>(255); }
    SUBCASE("forged eligibility") { artifact.crossings[0].ab.geometricallyEligible = false; }
    SUBCASE("uncanonical geometry") { std::swap(artifact.crossings[0].a, artifact.crossings[0].b); }
    const auto compiled = compileGraph(artifact);
    CHECK(compiled.status == StageStatus::InvalidInput);
    CHECK_FALSE(compiled.value);
}

TEST_CASE("v2 retains nearby distinct geometry and deterministic exact duplicates") {
    auto near = candidate();
    near.a.position.x = 0.0001f;
    const auto result = validateCrossings(topology(), config(), {near, candidate()});
    REQUIRE(result.value);
    CHECK(result.value->crossings.size() == 2);
    auto artifact = *result.value;
    artifact.crossings.push_back(artifact.crossings.front());
    std::reverse(artifact.crossings.begin(), artifact.crossings.end());
    const auto compiled = compileGraph(artifact);
    REQUIRE(compiled.value);
    CHECK(compiled.stats.exactDuplicates == 1);
    CHECK(compiled.stats.compiledCrossings == 2);
    CHECK((**compiled.value).crossings()[0].crossing.a.position.x == 0);
    CHECK((**compiled.value).offsets() == std::vector<std::size_t>{0, 2, 4});
}

TEST_CASE("v2 outbound policy remains an island property across different crossings") {
    auto near = candidate();
    near.a.position.x = 0.25f;
    ValidationOptions options;
    options.outboundPolicy = [](IslandId) { return true; };
    auto result = validateCrossings(topology(), config(), {candidate(), near}, options);
    REQUIRE(result.value);
    result.value->crossings[1].ab.policyAllowed = false;
    const auto compiled = compileGraph(*result.value);
    CHECK(compiled.status == StageStatus::InvalidInput);
    CHECK_FALSE(compiled.value);
}

TEST_CASE("v2 unversioned custom semantics disable persistent reuse") {
    auto mesh = topology();
    mesh.customPolygonPolicy = true;
    ValidationOptions options;
    options.validator = [](const ValidationRequest&) { return ValidationResult{}; };
    options.outboundPolicy = [](IslandId) { return true; };
    auto result = validateCrossings(mesh, config(), {candidate()}, options);
    REQUIRE(result.value);
    auto compiled = compileGraph(*result.value);
    REQUIRE(compiled.value);
    CHECK_FALSE((**compiled.value).persistentReuseEligible());
    auto& identity = result.value->topology.identity;
    identity.polygonPolicy = 3;
    identity.outboundPolicy = 4;
    identity.validator = 5;
    identity.environment = 6;
    compiled = compileGraph(*result.value);
    REQUIRE(compiled.value);
    CHECK((**compiled.value).persistentReuseEligible());
}

TEST_CASE("v2 empty completed stages remain distinguishable from failure") {
    const auto result = validateCrossings({}, config(), {});
    REQUIRE(result.value);
    const auto compiled = compileGraph(*result.value);
    REQUIRE(compiled.value);
    CHECK((**compiled.value).offsets() == std::vector<std::size_t>{0});
    CHECK((**compiled.value).crossings().empty());
    static_assert(std::is_const_v<typename std::shared_ptr<const CompiledGraph>::element_type>);
}

TEST_CASE("v2 geometric eligibility follows scale and translation without a sphere cutoff") {
    auto pair = candidate();
    pair.b.position = {5, 3, 0};
    auto limits = config();
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        auto transformed = pair;
        for (auto* anchor : {&transformed.a, &transformed.b}) {
            anchor->position.x = (anchor->position.x + 10) * scale;
            anchor->position.y = (anchor->position.y + 10) * scale;
            anchor->position.z = 10 * scale;
        }
        limits.sampleSpacing = scale;
        // Keep this test away from float-rounding at the exact capability boundary.
        limits.maxHorizontalGap = 5.01f * scale;
        limits.maxClimb = 3.01f * scale;
        limits.maxDrop = 6 * scale;
        const auto result = validateCrossings(topology(), limits, {transformed});
        REQUIRE(result.value);
        CHECK(compileGraph(*result.value).stats.compiledDirections == 2);
    }
}
} // namespace
