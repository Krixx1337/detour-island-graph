#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>
#include <detour_island_graph/v2/Serialization.h>
#include <DetourNavMeshBuilder.h>

#include <array>
#include <memory>
#include <sstream>
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
    params.bmax[0] = 12;
    params.bmax[1] = 5;
    params.bmax[2] = 12;
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

std::shared_ptr<const CompiledGraph> buildPair(const Mesh& mesh, std::uint64_t meshId = 0) {
    BuildInput input;
    input.navMesh = mesh.get();
    input.identity.mesh = meshId;
    input.identity.movementProfile = 7;
    const auto result = buildGraph(input, {2, 3, 2, 4, 0, 0});
    REQUIRE(result.status == StageStatus::Success);
    REQUIRE(result.compilation.value);
    return *result.compilation.value;
}

bool sameDirection(const DirectionResult& a, const DirectionResult& b) {
    return a.geometricallyEligible == b.geometricallyEligible && a.policyAllowed == b.policyAllowed &&
        a.validation.state == b.validation.state && a.validation.reason == b.validation.reason;
}

void checkEqualGraphs(const CompiledGraph& a, const CompiledGraph& b) {
    CHECK(a.identity().mesh == b.identity().mesh);
    CHECK(a.identity().movementProfile == b.identity().movementProfile);
    CHECK(a.identity().unitsPerMeter == b.identity().unitsPerMeter);
    CHECK(a.discovery().sampleSpacing == b.discovery().sampleSpacing);
    CHECK(a.discovery().maxHorizontalGap == b.discovery().maxHorizontalGap);
    CHECK(a.discovery().maxClimb == b.discovery().maxClimb);
    CHECK(a.discovery().maxDrop == b.discovery().maxDrop);
    CHECK(a.policy() == b.policy());
    CHECK(a.customPolygonPolicy() == b.customPolygonPolicy());
    CHECK(a.validatorSupplied() == b.validatorSupplied());
    CHECK(a.customOutboundPolicy() == b.customOutboundPolicy());
    CHECK(a.polygonIslands() == b.polygonIslands());
    CHECK(a.offsets() == b.offsets());
    CHECK(a.traversals().size() == b.traversals().size());
    for (std::size_t i = 0; i < a.traversals().size(); ++i) {
        CHECK(a.traversals()[i].crossing == b.traversals()[i].crossing);
        CHECK(a.traversals()[i].reverse == b.traversals()[i].reverse);
    }
    REQUIRE(a.crossings().size() == b.crossings().size());
    for (std::size_t i = 0; i < a.crossings().size(); ++i) {
        CHECK(a.crossings()[i].crossing.a.island == b.crossings()[i].crossing.a.island);
        CHECK(a.crossings()[i].crossing.a.polygon == b.crossings()[i].crossing.a.polygon);
        CHECK(a.crossings()[i].crossing.a.position.x == b.crossings()[i].crossing.a.position.x);
        CHECK(a.crossings()[i].crossing.b.island == b.crossings()[i].crossing.b.island);
        CHECK(a.crossings()[i].crossing.b.polygon == b.crossings()[i].crossing.b.polygon);
        CHECK(sameDirection(a.crossings()[i].crossing.ab, b.crossings()[i].crossing.ab));
        CHECK(sameDirection(a.crossings()[i].crossing.ba, b.crossings()[i].crossing.ba));
        CHECK(a.crossings()[i].traversableAB == b.crossings()[i].traversableAB);
        CHECK(a.crossings()[i].traversableBA == b.crossings()[i].traversableBA);
    }
}
} // namespace

TEST_CASE("V2 serialization round-trips compiled graphs deterministically") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    auto graph = buildPair(mesh, 42);
    REQUIRE_FALSE(graph->crossings().empty());

    std::ostringstream first, second;
    CHECK(GraphSerializer::write(first, *graph) == SerializationStatus::Success);
    CHECK(GraphSerializer::write(second, *graph) == SerializationStatus::Success);
    CHECK(first.str() == second.str());

    std::istringstream input(first.str());
    const auto decoded = GraphSerializer::read(input);
    CHECK(decoded.status == SerializationStatus::Success);
    REQUIRE(decoded.graph);
    checkEqualGraphs(*graph, *decoded.graph);
    // Identity survives so hosts can compare cache keys before reuse.
    CHECK(decoded.graph->identity().mesh == 42);

    // Validated-only graphs round-trip with validator provenance intact.
    BuildInput buildInput;
    buildInput.navMesh = mesh.get();
    buildInput.identity.mesh = 42;
    buildInput.identity.movementProfile = 7;
    buildInput.identity.validator = 11;
    buildInput.identity.environment = 13;
    ValidationOptions validation;
    validation.validator = [](const ValidationRequest&) {
        return ValidationResult{ValidationState::Valid, 0};
    };
    const auto strict =
        buildGraph(buildInput, {2, 3, 2, 4, 0, 0}, validation, {CompilePolicy::ValidatedOnly, {}});
    REQUIRE(strict.status == StageStatus::Success);
    std::ostringstream strictBytes;
    REQUIRE(GraphSerializer::write(strictBytes, **strict.compilation.value) ==
        SerializationStatus::Success);
    std::istringstream strictInput(strictBytes.str());
    const auto strictDecoded = GraphSerializer::read(strictInput);
    CHECK(strictDecoded.status == SerializationStatus::Success);
    REQUIRE(strictDecoded.graph);
    CHECK(strictDecoded.graph->validatorSupplied());
    CHECK(strictDecoded.graph->policy() == CompilePolicy::ValidatedOnly);
}

TEST_CASE("V2 serialization round-trips empty graphs") {
    auto mesh = makeMesh({{0, 0, 2, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    input.polygonFilter = [](dtPolyRef, const dtMeshTile&, const dtPoly&) { return false; };
    const auto built = buildGraph(input, {2, 3, 2, 4, 0, 0});
    REQUIRE(built.status == StageStatus::Success);
    std::ostringstream bytes;
    REQUIRE(GraphSerializer::write(bytes, **built.compilation.value) == SerializationStatus::Success);
    std::istringstream stream(bytes.str());
    const auto decoded = GraphSerializer::read(stream);
    CHECK(decoded.status == SerializationStatus::Success);
    REQUIRE(decoded.graph);
    CHECK(decoded.graph->crossings().empty());
    CHECK(decoded.graph->offsets() == std::vector<std::size_t>{0});
}

TEST_CASE("V2 serialization rejects foreign corrupt and hostile input") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    auto graph = buildPair(mesh);
    std::ostringstream bytes;
    REQUIRE(GraphSerializer::write(bytes, *graph) == SerializationStatus::Success);
    const std::string blob = bytes.str();
    REQUIRE(blob.size() > 200);

    const std::string v1blob("DIG1\x04\x00\x00\x00", 8);
    std::istringstream v1(v1blob);
    CHECK(GraphSerializer::read(v1).status == SerializationStatus::InvalidMagic);

    auto versioned = blob;
    versioned[4] = 2;
    std::istringstream versionStream(versioned);
    CHECK(GraphSerializer::read(versionStream).status == SerializationStatus::UnsupportedVersion);

    std::istringstream truncated(blob.substr(0, blob.size() / 2));
    CHECK(GraphSerializer::read(truncated).status == SerializationStatus::MalformedData);

    // First crossing islandA low byte sits after the 112-byte header and
    // 2x12-byte polygon entries on this mesh; forcing it out of range must fail.
    auto badIsland = blob;
    badIsland[112 + 24] = 9;
    std::istringstream islandStream(badIsland);
    CHECK(GraphSerializer::read(islandStream).status == SerializationStatus::MalformedData);

    // Direction state byte follows the two 24-byte anchors plus two flag bytes.
    auto badState = blob;
    badState[112 + 24 + 24 + 2] = 3;
    std::istringstream stateStream(badState);
    CHECK(GraphSerializer::read(stateStream).status == SerializationStatus::MalformedData);

    DecodeOptions tight;
    tight.limits.maxIslandCount = 1;
    std::istringstream limitStream(blob);
    CHECK(GraphSerializer::read(limitStream, tight).status == SerializationStatus::MalformedData);

    DecodeOptions tiny;
    tiny.limits.maxAllocationBytes = 16;
    std::istringstream budgetStream(blob);
    CHECK(GraphSerializer::read(budgetStream, tiny).status == SerializationStatus::MalformedData);

    DecodeOptions canceled;
    canceled.canceled = [] { return true; };
    std::istringstream cancelStream(blob);
    CHECK(GraphSerializer::read(cancelStream, canceled).status == SerializationStatus::Canceled);
}
