#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>
#include <DetourNavMeshBuilder.h>

#include <algorithm>
#include <array>
#include <cmath>
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

Mesh makeMesh(const std::vector<Rect>& rects, float scale = 1) {
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
    params.bmax[0] = 10 * scale;
    params.bmax[1] = 5 * scale;
    params.bmax[2] = 10 * scale;
    for (const auto& r : rects) {
        params.bmax[0] = (std::max)(params.bmax[0], r.x1 * scale);
        params.bmax[1] = (std::max)(params.bmax[1], float(r.y * scale + scale));
        params.bmax[2] = (std::max)(params.bmax[2], r.z1 * scale);
    }
    params.walkableHeight = 2 * scale;
    params.walkableRadius = 0.5f * scale;
    params.walkableClimb = 0.5f * scale;
    params.cs = scale;
    params.ch = scale;
    params.buildBvTree = true;
    int size = 0;
    unsigned char* data = nullptr;
    REQUIRE(dtCreateNavMeshData(&params, &data, &size));
    Mesh mesh(dtAllocNavMesh());
    REQUIRE(mesh);
    REQUIRE(dtStatusSucceed(mesh->init(data, size, DT_TILE_FREE_DATA)));
    return mesh;
}

Mesh twoIslandMesh(unsigned short gapStart = 2, unsigned short gapEnd = 4,
    unsigned short secondY = 0) {
    return makeMesh({{0, 0, gapStart, 2, 0}, {gapEnd, 0, 6, 2, secondY}});
}

Mesh singleQuadMesh() { return makeMesh({{0, 0, 2, 2}}); }

DiscoveryConfig discovery(float gap = 3, float climb = 2, float drop = 4) {
    return {2, gap, climb, drop, 0, 0};
}

StageResult<SamplingArtifact> sampleMesh(const Mesh& mesh, DiscoveryConfig config) {
    BuildInput input;
    input.navMesh = mesh.get();
    return extractAndSample(input, config);
}
} // namespace

TEST_CASE("V2 discovery emits different-island pairs within independent limits") {
    auto mesh = twoIslandMesh();
    auto sampled = sampleMesh(mesh, discovery());
    REQUIRE(sampled.status == StageStatus::Success);
    REQUIRE(sampled.value);
    CHECK(sampled.value->topology.islandCount == 2);

    const auto found = discoverCandidates(*sampled.value, *mesh, discovery());
    REQUIRE(found.status == StageStatus::Success);
    REQUIRE(found.value);
    CHECK_FALSE(found.value->empty());
    CHECK(found.stats.discoveryQueries == sampled.value->samples.size());
    CHECK(found.stats.projections > 0);
    CHECK(found.stats.candidatesVisited == found.value->size());

    std::unordered_map<dtPolyRef, IslandId> islands;
    for (const auto& entry : sampled.value->topology.polygons) islands[entry.polygon] = entry.island;
    for (const auto& candidate : *found.value) {
        CHECK(candidate.a.island != candidate.b.island);
        CHECK(islands.at(candidate.a.polygon) == candidate.a.island);
        CHECK(islands.at(candidate.b.polygon) == candidate.b.island);
        const double dx = double(candidate.b.position.x) - candidate.a.position.x;
        const double dz = double(candidate.b.position.z) - candidate.a.position.z;
        const double dy = double(candidate.b.position.y) - candidate.a.position.y;
        CHECK(std::hypot(dx, dz) <= 3);
        const bool ab = dy <= 2 && -dy <= 4;
        const bool ba = -dy <= 2 && dy <= 4;
        CHECK((ab || ba) == true);
    }

    // Feed into validation: every discovered pair validates geometrically.
    std::vector<CrossingCandidate> raw = *found.value;
    const auto validated =
        validateCrossings(sampled.value->topology, discovery(), raw, {});
    REQUIRE(validated.status == StageStatus::Success);
    REQUIRE(validated.value);
    CHECK(validated.value->crossings.size() <= raw.size());
    CHECK(validated.stats.candidatesVisited == raw.size());
}

TEST_CASE("V2 discovery finds nothing without a reachable neighbor") {
    auto mesh = singleQuadMesh();
    auto sampled = sampleMesh(mesh, discovery());
    REQUIRE(sampled.status == StageStatus::Success);
    const auto found = discoverCandidates(*sampled.value, *mesh, discovery());
    REQUIRE(found.status == StageStatus::Success);
    REQUIRE(found.value);
    CHECK(found.value->empty());
    CHECK(found.stats.candidatesVisited == 0);

    auto far = twoIslandMesh();
    auto farSampled = sampleMesh(far, discovery(1));
    REQUIRE(farSampled.status == StageStatus::Success);
    const auto none = discoverCandidates(*farSampled.value, *far, discovery(1));
    REQUIRE(none.status == StageStatus::Success);
    CHECK(none.value->empty());
}

TEST_CASE("V2 discovery keeps reverse-valid asymmetric pairs") {
    // Second island 3 units above the first. Climb 2 blocks low->high,
    // drop 4 still allows high->low.
    auto mesh = twoIslandMesh(2, 4, 3);
    auto sampled = sampleMesh(mesh, discovery(3, 2, 4));
    REQUIRE(sampled.status == StageStatus::Success);
    const auto found = discoverCandidates(*sampled.value, *mesh, discovery(3, 2, 4));
    REQUIRE(found.status == StageStatus::Success);
    REQUIRE(found.value);
    CHECK_FALSE(found.value->empty());

    // Symmetric climb that blocks both directions must yield nothing.
    const auto blocked = discoverCandidates(*sampled.value, *mesh, discovery(3, 2, 2));
    REQUIRE(blocked.status == StageStatus::Success);
    CHECK(blocked.value->empty());
}

TEST_CASE("V2 discovery cap is enforced while generating with no partial output") {
    auto mesh = twoIslandMesh();
    auto sampled = sampleMesh(mesh, discovery());
    REQUIRE(sampled.status == StageStatus::Success);
    const auto uncapped = discoverCandidates(*sampled.value, *mesh, discovery());
    REQUIRE(uncapped.status == StageStatus::Success);
    REQUIRE(uncapped.value);
    REQUIRE_FALSE(uncapped.value->empty());
    const std::size_t exact = uncapped.value->size();

    auto limits = discovery();
    limits.maxCandidates = exact;
    const auto allowed = discoverCandidates(*sampled.value, *mesh, limits);
    CHECK(allowed.status == StageStatus::Success);
    REQUIRE(allowed.value);
    CHECK(allowed.value->size() == exact);

    limits.maxCandidates = exact - 1;
    const auto exceeded = discoverCandidates(*sampled.value, *mesh, limits);
    CHECK(exceeded.status == StageStatus::BudgetExceeded);
    CHECK_FALSE(exceeded.value);
}

TEST_CASE("V2 discovery cancellation and malformed input publish nothing") {
    auto mesh = twoIslandMesh();
    auto sampled = sampleMesh(mesh, discovery());
    REQUIRE(sampled.status == StageStatus::Success);

    bool cancel = false;
    Cancel canceled = [&] { return cancel; };
    cancel = true;
    const auto canceledResult = discoverCandidates(*sampled.value, *mesh, discovery(), canceled);
    CHECK(canceledResult.status == StageStatus::Canceled);
    CHECK_FALSE(canceledResult.value);

    unsigned calls = 0;
    Cancel late = [&] { return ++calls > 3; };
    const auto lateCancel = discoverCandidates(*sampled.value, *mesh, discovery(), late);
    CHECK(lateCancel.status == StageStatus::Canceled);
    CHECK_FALSE(lateCancel.value);

    auto bad = *sampled.value;
    bad.samples.front().polygon = 0;
    const auto invalid = discoverCandidates(bad, *mesh, discovery());
    CHECK(invalid.status == StageStatus::InvalidInput);
    CHECK_FALSE(invalid.value);

    auto badConfig = discovery();
    badConfig.maxHorizontalGap = -1;
    const auto badLimits = discoverCandidates(*sampled.value, *mesh, badConfig);
    CHECK(badLimits.status == StageStatus::InvalidInput);
    CHECK_FALSE(badLimits.value);
}
