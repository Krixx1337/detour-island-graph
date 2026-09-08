#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>
#include <DetourNavMeshBuilder.h>
#ifdef DIG_NATIVE_TRANSFERS
#include <detour_island_graph/v2/NativeTransfers.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <tuple>
#include <vector>

#ifdef DIG_NATIVE_TRANSFERS
namespace {
detour_island_graph::v2::NativeTransferOptions nativeLimits() {
    return {10000,100000,128,128,256,1000,.001f,{}};
}
}
#endif

namespace {
using namespace detour_island_graph::v2;
constexpr unsigned short border = 0xffff;

struct Rect {
    unsigned short x0, z0, x1, z1, y = 0;
    std::array<unsigned short, 4> neighbors{border, border, border, border};
};

struct MeshDeleter { void operator()(dtNavMesh* mesh) const { dtFreeNavMesh(mesh); } };
using Mesh = std::unique_ptr<dtNavMesh, MeshDeleter>;

unsigned char* tileData(const std::vector<Rect>& rects, int tileX, float scale,
    Point translation, int& size, bool offMesh = false) {
    std::vector<unsigned short> verts, polys, flags(rects.size(), 1);
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
    params.tileX = tileX;
    params.bmin[0] = translation.x + tileX * 10 * scale;
    params.bmin[1] = translation.y;
    params.bmin[2] = translation.z;
    params.bmax[0] = params.bmin[0] + 10 * scale;
    params.bmax[1] = params.bmin[1] + scale;
    params.bmax[2] = params.bmin[2] + 10 * scale;
    for (const auto& r : rects) {
        params.bmax[0] = (std::max)(params.bmax[0], params.bmin[0] + r.x1 * scale);
        params.bmax[1] = (std::max)(params.bmax[1], params.bmin[1] + r.y * scale);
        params.bmax[2] = (std::max)(params.bmax[2], params.bmin[2] + r.z1 * scale);
    }
    params.walkableHeight = 2 * scale;
    params.walkableRadius = 0.5f * scale;
    params.walkableClimb = 0.5f * scale;
    params.cs = scale;
    params.ch = scale;
    params.buildBvTree = true;
    const float endpoints[] = {1, 0, 1, 9, 0, 1};
    const float radius = 0.5f;
    const unsigned char direction = DT_OFFMESH_CON_BIDIR, area = 0;
    const unsigned short flag = 1;
    if (offMesh) {
        params.offMeshConCount = 1;
        params.offMeshConVerts = endpoints;
        params.offMeshConRad = &radius;
        params.offMeshConDir = &direction;
        params.offMeshConAreas = &area;
        params.offMeshConFlags = &flag;
    }
    unsigned char* data = nullptr;
    REQUIRE(dtCreateNavMeshData(&params, &data, &size));
    return data;
}

Mesh singleMesh(const std::vector<Rect>& rects, float scale = 1, Point translation = {}, bool offMesh = false) {
    Mesh mesh(dtAllocNavMesh());
    REQUIRE(mesh);
    int size = 0;
    auto* data = tileData(rects, 0, scale, translation, size, offMesh);
    REQUIRE(dtStatusSucceed(mesh->init(data, size, DT_TILE_FREE_DATA)));
    return mesh;
}

Mesh partialMesh(bool reverseOrder = false) {
    dtNavMeshParams params{};
    params.tileWidth = params.tileHeight = 10;
    params.maxTiles = 2;
    params.maxPolys = 4;
    Mesh mesh(dtAllocNavMesh());
    REQUIRE(mesh);
    REQUIRE(dtStatusSucceed(mesh->init(&params)));
    Rect source{0, 0, 10, 10};
    source.neighbors[1] = DT_EXT_LINK | 2;
    Rect first{0, 2, 4, 4}, second{0, 6, 4, 8};
    first.neighbors[3] = second.neighbors[3] = DT_EXT_LINK | 0;
    for (int n = 0; n < 2; ++n) {
        const int x = reverseOrder ? 1 - n : n;
        int size = 0;
        auto* data = tileData(x == 0 ? std::vector<Rect>{source} : std::vector<Rect>{first, second},
            x, 1, {}, size);
        REQUIRE(dtStatusSucceed(mesh->addTile(data, size, DT_TILE_FREE_DATA, 0, nullptr)));
    }
    return mesh;
}

DiscoveryConfig config(float spacing = 2) { return {spacing, 3, 2, 4, 0, 0}; }

StageResult<SamplingArtifact> sample(const Mesh& mesh, DiscoveryConfig settings = config()) {
    BuildInput input;
    input.navMesh = mesh.get();
    return extractAndSample(input, settings);
}

std::vector<BoundaryInterval> sourcePortal(const SamplingArtifact& artifact, const Mesh& mesh) {
    std::vector<BoundaryInterval> intervals;
    for (const auto& interval : artifact.intervals) {
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        REQUIRE(dtStatusSucceed(mesh->getTileAndPolyByRef(interval.polygon, &tile, &poly)));
        if (tile->header->x == 0 && interval.edge == 1) intervals.push_back(interval);
    }
    return intervals;
}

auto sampleKeys(const SamplingArtifact& artifact, const Mesh& mesh) {
    std::vector<std::tuple<IslandId, float, float, float, int, int>> keys;
    for (const auto& anchor : artifact.samples) {
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        REQUIRE(dtStatusSucceed(mesh->getTileAndPolyByRef(anchor.polygon, &tile, &poly)));
        keys.emplace_back(anchor.island, anchor.position.x, anchor.position.y, anchor.position.z,
            tile->header->x, int(poly - tile->polys));
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}
} // namespace

TEST_CASE("V2 sampling includes long edge endpoints and bounds spacing without island quotas") {
    auto mesh = singleMesh({{0, 0, 1000, 2}, {1002, 0, 1003, 1}});
    auto result = sample(mesh, config(2));
    REQUIRE(result.status == StageStatus::Success);
    REQUIRE(result.value);
    CHECK(result.stats.islands == 2);
    CHECK(result.stats.boundaryIntervals == 8);
    CHECK(result.stats.samples == 1006);
    CHECK(result.stats.sampleDuplicates == 8);
    std::vector<float> edge;
    for (const auto& anchor : result.value->samples)
        if (anchor.island == 0 && anchor.position.z == 0) edge.push_back(anchor.position.x);
    std::sort(edge.begin(), edge.end());
    REQUIRE(edge.size() == 501);
    CHECK(edge.front() == 0);
    CHECK(edge.back() == 1000);
    for (std::size_t i = 1; i < edge.size(); ++i) CHECK(edge[i] - edge[i - 1] <= 2);
}

TEST_CASE("V2 sampling subtracts every external portal interval and joins every eligible neighbor") {
    auto mesh = partialMesh();
    auto result = sample(mesh);
    REQUIRE(result.status == StageStatus::Success);
    CHECK(result.stats.eligiblePolygons == 3);
    CHECK(result.stats.islands == 1);
    auto intervals = sourcePortal(*result.value, mesh);
    REQUIRE(intervals.size() == 3);
    CHECK(intervals[0].start.z == doctest::Approx(0));
    CHECK(intervals[0].finish.z == doctest::Approx(2));
    CHECK(intervals[1].start.z == doctest::Approx(4));
    CHECK(intervals[1].finish.z == doctest::Approx(6));
    CHECK(intervals[2].start.z == doctest::Approx(8));
    CHECK(intervals[2].finish.z == doctest::Approx(10));
    CHECK(intervals[1].begin == doctest::Approx(102.0 / 255));
    CHECK(intervals[1].end == doctest::Approx(153.0 / 255));
}

TEST_CASE("V2 sampling unions overlapping portal intervals rather than emitting false gaps") {
    auto mesh = partialMesh();
    auto* tile = const_cast<dtMeshTile*>(mesh->getTileAt(0, 0, 0));
    REQUIRE(tile);
    auto linkIndex = tile->polys[0].firstLink;
    REQUIRE(linkIndex != DT_NULL_LINK);
    tile->links[linkIndex].bmin = 51;
    tile->links[linkIndex].bmax = 153;
    linkIndex = tile->links[linkIndex].next;
    REQUIRE(linkIndex != DT_NULL_LINK);
    tile->links[linkIndex].bmin = 102;
    tile->links[linkIndex].bmax = 204;
    auto result = sample(mesh);
    REQUIRE(result.status == StageStatus::Success);
    auto intervals = sourcePortal(*result.value, mesh);
    REQUIRE(intervals.size() == 2);
    CHECK(intervals[0].finish.z == doctest::Approx(2));
    CHECK(intervals[1].start.z == doctest::Approx(8));
}

TEST_CASE("V2 sampling exposes portals leading to filtered polygons and invokes filter once") {
    auto mesh = partialMesh();
    BuildInput input;
    input.navMesh = mesh.get();
    int calls = 0;
    input.polygonFilter = [&](dtPolyRef, const dtMeshTile& tile, const dtPoly& poly) {
        ++calls;
        return tile.header->x == 0 || &poly != tile.polys;
    };
    auto result = extractAndSample(input, config());
    REQUIRE(result.status == StageStatus::Success);
    CHECK(calls == 3);
    CHECK(result.stats.eligiblePolygons == 2);
    CHECK(result.stats.islands == 1);
    CHECK(result.value->topology.customPolygonPolicy);
    auto intervals = sourcePortal(*result.value, mesh);
    REQUIRE(intervals.size() == 2);
    CHECK(intervals[0].finish.z == doctest::Approx(6));
    CHECK(intervals[1].start.z == doctest::Approx(8));
}

TEST_CASE("V2 sampling labels islands and owns exact duplicate samples independently of tile load order") {
    auto a = partialMesh(false), b = partialMesh(true);
    const auto ra = sample(a), rb = sample(b);
    REQUIRE(ra.status == StageStatus::Success);
    REQUIRE(rb.status == StageStatus::Success);
    CHECK(sampleKeys(*ra.value, a) == sampleKeys(*rb.value, b));
    CHECK(ra.stats.samples == rb.stats.samples);
}

TEST_CASE("V2 sampling excludes off mesh actions even when polygon filter accepts everything") {
    auto mesh = singleMesh({{0, 0, 2, 2}, {8, 0, 10, 2}}, 1, {}, true);
    REQUIRE(mesh->getTileAt(0, 0, 0)->header->offMeshConCount == 1);
    BuildInput input;
    input.navMesh = mesh.get();
    int calls = 0;
    input.polygonFilter = [&](dtPolyRef, const dtMeshTile&, const dtPoly& poly) {
        ++calls;
        CHECK(poly.getType() == DT_POLYTYPE_GROUND);
        return true;
    };
    auto result = extractAndSample(input, config());
    REQUIRE(result.status == StageStatus::Success);
    CHECK(result.stats.islands == 2);
    CHECK(result.stats.eligiblePolygons == 2);
    CHECK(calls == 2);
}

TEST_CASE("V2 sampling internal ground connectivity and filter boundaries survive retessellation") {
    Rect left{0, 0, 5, 2}, right{5, 0, 10, 2};
    left.neighbors[1] = 1;
    right.neighbors[3] = 0;
    auto split = singleMesh({left, right}), whole = singleMesh({{0, 0, 10, 2}});
    auto a = sample(split, config(1)), b = sample(whole, config(1));
    REQUIRE(a.status == StageStatus::Success);
    REQUIRE(b.status == StageStatus::Success);
    CHECK(a.stats.islands == 1);
    CHECK(a.stats.samples == b.stats.samples);
    for (const auto& anchor : a.value->samples) {
        CHECK(std::any_of(b.value->samples.begin(), b.value->samples.end(), [&](const Anchor& other) {
            return anchor.position.x == other.position.x && anchor.position.y == other.position.y &&
                anchor.position.z == other.position.z;
        }));
    }
    BuildInput input;
    input.navMesh = split.get();
    input.polygonFilter = [](dtPolyRef, const dtMeshTile& tile, const dtPoly& poly) { return &poly == tile.polys; };
    auto filtered = extractAndSample(input, config(1));
    REQUIRE(filtered.status == StageStatus::Success);
    CHECK(filtered.stats.boundaryIntervals == 4);
    CHECK(filtered.stats.samples == 14);
}

TEST_CASE("V2 sampling cap applies to unique samples and never publishes partial output") {
    auto mesh = singleMesh({{0, 0, 10, 2}});
    auto settings = config(2);
    settings.maxSamples = 12;
    auto exact = sample(mesh, settings);
    REQUIRE(exact.status == StageStatus::Success);
    CHECK(exact.stats.samples == 12);
    CHECK(exact.stats.sampleAttempts > exact.stats.samples);
    settings.maxSamples = 11;
    auto exceeded = sample(mesh, settings);
    CHECK(exceeded.status == StageStatus::BudgetExceeded);
    CHECK_FALSE(exceeded.value);
    CHECK(exceeded.stats.samples == 11);
}

TEST_CASE("V2 sampling cancellation during work and callback exceptions leave no artifact") {
    auto mesh = singleMesh({{0, 0, 1000, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    unsigned calls = 0;
    input.canceled = [&] { return ++calls > 200; };
    auto canceled = extractAndSample(input, config(0.1f));
    CHECK(canceled.status == StageStatus::Canceled);
    CHECK(canceled.stats.samples > 0);
    CHECK_FALSE(canceled.value);
    input.canceled = {};
    input.polygonFilter = [](dtPolyRef, const dtMeshTile&, const dtPoly&) -> bool { throw std::runtime_error("filter"); };
    auto failed = extractAndSample(input, config());
    CHECK(failed.status == StageStatus::CallbackFailed);
    CHECK_FALSE(failed.value);
}

TEST_CASE("V2 sampling scales with explicit spacing and does not couple density to drop range") {
    auto original = singleMesh({{0, 0, 10, 2}});
    auto transformed = singleMesh({{0, 0, 10, 2}}, 8, {512, -32, 256});
    auto a = sample(original, config(2));
    auto settings = config(16);
    settings.maxDrop = 10000;
    auto b = sample(transformed, settings);
    REQUIRE(a.status == StageStatus::Success);
    REQUIRE(b.status == StageStatus::Success);
    REQUIRE(a.value->samples.size() == b.value->samples.size());
    for (std::size_t i = 0; i < a.value->samples.size(); ++i) {
        const auto p = a.value->samples[i].position, q = b.value->samples[i].position;
        CHECK(q.x == doctest::Approx(p.x * 8 + 512));
        CHECK(q.y == doctest::Approx(p.y * 8 - 32));
        CHECK(q.z == doctest::Approx(p.z * 8 + 256));
    }
}

TEST_CASE("V2 sampling rejects invalid settings precision loss and one way native ground") {
    BuildInput input;
    auto settings = config();
    SUBCASE("missing mesh") {
        CHECK(extractAndSample(input, settings).status == StageStatus::InvalidInput);
        return;
    }
    auto mesh = singleMesh({{0, 0, 10, 2}});
    input.navMesh = mesh.get();
    SUBCASE("zero spacing") { settings.sampleSpacing = 0; }
    SUBCASE("non finite spacing") { settings.sampleSpacing = std::numeric_limits<float>::infinity(); }
    SUBCASE("negative reach") { settings.maxHorizontalGap = -1; }
    SUBCASE("invalid scale") { input.identity.unitsPerMeter = 0; }
    SUBCASE("unrepresentable samples") {
        mesh = singleMesh({{0, 0, 10, 2}}, 1, {16777216, 0, 0});
        input.navMesh = mesh.get();
        settings.sampleSpacing = 0.5f;
    }
    SUBCASE("one way ground") {
        Rect left{0, 0, 5, 2}, right{5, 0, 10, 2};
        left.neighbors[1] = 1;
        mesh = singleMesh({left, right});
        input.navMesh = mesh.get();
    }
    SUBCASE("all filtered is successful empty") {
        input.polygonFilter = [](dtPolyRef, const dtMeshTile&, const dtPoly&) { return false; };
        const auto result = extractAndSample(input, settings);
        REQUIRE(result.status == StageStatus::Success);
        REQUIRE(result.value);
        CHECK(result.value->topology.islandCount == 0);
        CHECK(result.value->samples.empty());
        return;
    }
    SUBCASE("valid input") { return; }
    const auto result = extractAndSample(input, settings);
    CHECK(result.status == StageStatus::InvalidInput);
    CHECK_FALSE(result.value);
}

TEST_CASE("V2 sampling distinguishes complete portal coverage from an unresolved seam") {
    auto mesh = partialMesh();
    auto* tile = const_cast<dtMeshTile*>(mesh->getTileAt(0, 0, 0));
    REQUIRE(tile);
    auto linkIndex = tile->polys[0].firstLink;
    REQUIRE(linkIndex != DT_NULL_LINK);
    tile->links[linkIndex].bmin = 0;
    tile->links[linkIndex].bmax = 255;
    auto complete = sample(mesh);
    REQUIRE(complete.status == StageStatus::Success);
    CHECK(sourcePortal(*complete.value, mesh).empty());

    Rect isolated{0, 0, 10, 10};
    isolated.neighbors[1] = DT_EXT_LINK | 2;
    auto seam = singleMesh({isolated});
    auto exposed = sample(seam);
    REQUIRE(exposed.status == StageStatus::Success);
    CHECK(exposed.stats.boundaryIntervals == 4);
}

TEST_CASE("V2 sampling keeps coincident positions on different islands separate") {
    auto mesh = singleMesh({{0, 0, 2, 2}, {0, 0, 2, 2}});
    auto result = sample(mesh);
    REQUIRE(result.status == StageStatus::Success);
    CHECK(result.stats.islands == 2);
    CHECK(result.stats.samples == 8);
    for (IslandId island : {0u, 1u}) {
        CHECK(std::count_if(result.value->samples.begin(), result.value->samples.end(),
            [&](const Anchor& a) { return a.island == island; }) == 4);
    }
}

TEST_CASE("V2 sampling follows three dimensional edge length on slopes") {
    auto mesh = singleMesh({{0, 0, 4, 2}});
    auto* tile = const_cast<dtMeshTile*>(mesh->getTileAt(0, 0, 0));
    // Coarse ground edge (0,0,0) -> (4,3,0) has length 5, not 4.
    tile->verts[3 + 1] = tile->verts[6 + 1] = 3;
    auto result = sample(mesh, config(1));
    REQUIRE(result.status == StageStatus::Success);
    std::vector<Point> edge;
    for (const auto& a : result.value->samples) if (a.position.z == 0) edge.push_back(a.position);
    std::sort(edge.begin(), edge.end(), [](Point a, Point b) { return a.x < b.x; });
    REQUIRE(edge.size() == 6);
    for (std::size_t i = 1; i < edge.size(); ++i)
        CHECK(std::hypot(double(edge[i].x) - edge[i - 1].x, double(edge[i].y) - edge[i - 1].y) ==
            doctest::Approx(1).epsilon(1e-6));
}

TEST_CASE("V2 topology can be reused for selected boundary sampling") {
    auto mesh = singleMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    int filterCalls = 0;
    input.polygonFilter = [&](dtPolyRef, const dtMeshTile&, const dtPoly&) {
        ++filterCalls;
        return true;
    };
    const auto topology = extractTopology(input);
    REQUIRE(topology.value);
    CHECK(topology.stats.samples == 0);
    CHECK(topology.stats.boundaryIntervals == 8);
    REQUIRE(topology.value->topology.metrics.size() == 2);
    CHECK(topology.value->topology.metrics[0].polygonCount == 1);
    CHECK(topology.value->topology.metrics[0].surfaceArea == doctest::Approx(4));
    CHECK(topology.value->topology.metrics[1].boundsMin.x == 4);
    CHECK(topology.value->topology.metrics[1].boundsMax.x == 6);

    SamplingOptions selection;
    selection.islands = std::vector<IslandId>{1, 1};
    auto result = sampleBoundaries(*topology.value, config(), selection);
    REQUIRE(result.value);
    CHECK(result.value->samples.size() == 4);
    CHECK(result.value->topology.polygons.size() == 2);
    for (const auto& anchor : result.value->samples) CHECK(anchor.island == 1);
    CHECK(filterCalls == 2);
    const auto discovered = discoverCandidates(*result.value, *mesh, config());
    REQUIRE(discovered.value);
    CHECK_FALSE(discovered.value->empty());
    for (const auto& candidate : *discovered.value) {
        CHECK(candidate.a.island == 1);
        CHECK(candidate.b.island == 0);
    }

    const auto all = sampleBoundaries(*topology.value, config());
    const auto wrapped = extractAndSample(input, config());
    REQUIRE(all.value);
    REQUIRE(wrapped.value);
    CHECK(sampleKeys(*all.value, mesh) == sampleKeys(*wrapped.value, mesh));
    selection.islands = std::vector<IslandId>{1, 0, 1};
    const auto reordered = sampleBoundaries(*topology.value, config(), selection);
    REQUIRE(reordered.value);
    CHECK(sampleKeys(*all.value, mesh) == sampleKeys(*reordered.value, mesh));

    selection.islands = std::vector<IslandId>{};
    result = sampleBoundaries(*topology.value, config(), selection);
    REQUIRE(result.value);
    CHECK(result.value->samples.empty());
    CHECK(result.value->topology.islandCount == 2);

    selection.islands = std::vector<IslandId>{2};
    result = sampleBoundaries(*topology.value, config(), selection);
    CHECK(result.status == StageStatus::InvalidInput);
    CHECK_FALSE(result.value);

    selection.islands.reset();
    auto limited = config();
    limited.maxSamples = 7;
    result = sampleBoundaries(*topology.value, limited, selection);
    CHECK(result.status == StageStatus::BudgetExceeded);
    CHECK_FALSE(result.value);
    limited.maxSamples = 8;
    CHECK(sampleBoundaries(*topology.value, limited, selection).status == StageStatus::Success);
    selection.canceled = [] { return true; };
    result = sampleBoundaries(*topology.value, config(), selection);
    CHECK(result.status == StageStatus::Canceled);
    CHECK_FALSE(result.value);
    selection.canceled = []() -> bool { throw std::runtime_error("cancel failure"); };
    CHECK(sampleBoundaries(*topology.value, config(), selection).status == StageStatus::CallbackFailed);
}

TEST_CASE("V2 topology metrics use 3D area and preserve polygon exclusions") {
    Rect left{0, 0, 4, 2}, right{4, 0, 8, 2};
    left.neighbors[1] = 1;
    right.neighbors[3] = 0;
    auto mesh = singleMesh({left, right});
    BuildInput input;
    input.navMesh = mesh.get();
    auto result = extractTopology(input);
    REQUIRE(result.value);
    REQUIRE(result.value->topology.metrics.size() == 1);
    CHECK(result.value->topology.metrics[0].polygonCount == 2);
    CHECK(result.value->topology.metrics[0].surfaceArea == doctest::Approx(16));
    const auto excluded = result.value->topology.polygons[1].polygon;
    input.polygonFilter = [=](dtPolyRef ref, const dtMeshTile&, const dtPoly&) { return ref != excluded; };
    auto* tile = const_cast<dtMeshTile*>(mesh->getTileAt(0, 0, 0));
    tile->verts[4] = tile->verts[7] = 3;
    result = extractTopology(input);
    REQUIRE(result.value);
    CHECK(result.value->topology.metrics[0].polygonCount == 1);
    CHECK(result.value->topology.metrics[0].surfaceArea == doctest::Approx(10));
    CHECK(result.value->topology.metrics[0].boundsMax.y == 3);
    CHECK(result.value->intervals.size() == 4);
}

TEST_CASE("V2 sampling rejects malformed edge data and cancellation after last filter") {
    auto mesh = singleMesh({{0, 0, 2, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    SUBCASE("non finite vertex") {
        auto* tile = const_cast<dtMeshTile*>(mesh->getTileAt(0, 0, 0));
        tile->verts[0] = std::numeric_limits<float>::quiet_NaN();
        const auto result = extractAndSample(input, config());
        CHECK(result.status == StageStatus::InvalidInput);
        CHECK_FALSE(result.value);
    }
    SUBCASE("filter triggers cancellation") {
        bool canceled = false;
        input.canceled = [&] { return canceled; };
        input.polygonFilter = [&](dtPolyRef, const dtMeshTile&, const dtPoly&) { canceled = true; return false; };
        const auto result = extractAndSample(input, config());
        CHECK(result.status == StageStatus::Canceled);
        CHECK_FALSE(result.value);
    }
}

TEST_CASE("V2 island policy failures never publish topology") {
    auto mesh = singleMesh({{0, 0, 2, 2}});
    BuildInput input;
    input.navMesh = mesh.get();
    bool cancel = false;
    input.canceled = [&] { return cancel; };
    StageStatus expected = StageStatus::CallbackFailed;
    SUBCASE("throwing policy") {
        input.islandPolicy = [](IslandId, const IslandMetrics&) -> IslandDomain {
            throw std::runtime_error("policy failed");
        };
    }
    SUBCASE("invalid policy result") {
        expected = StageStatus::InvalidInput;
        input.islandPolicy = [](IslandId, const IslandMetrics&) {
            return IslandDomain{static_cast<DomainState>(99), 0};
        };
    }
    SUBCASE("cancel after final decision") {
        expected = StageStatus::Canceled;
        input.islandPolicy = [&](IslandId, const IslandMetrics&) {
            cancel = true;
            return IslandDomain{};
        };
    }
    const auto result = extractTopology(input);
    CHECK(result.status == expected);
    CHECK_FALSE(result.value);
}


TEST_CASE("V2 bounded production preserves partial portals under tile allocation order") {
    const auto run = [](bool reverse) {
        auto mesh = partialMesh(reverse);
        BuildInput input;
        input.navMesh = mesh.get();
        // Excluding one destination leaves a partial seam and a remaining native component.
        input.polygonFilter = [](dtPolyRef, const dtMeshTile& tile, const dtPoly& poly) {
            return !(tile.header->x == 1 && &poly == &tile.polys[1]);
        };
        auto c = config();
        c.maxSamples = c.maxCandidates = 10000;
        ProductionBuildOptions options{{1024 * 1024, 100000, 100, 100, 100, 100, 100, 200}, 1, 1};
        const auto reference = buildGraph(input, c);
        const auto bounded = buildGraphBounded(input, c, options);
        REQUIRE(reference.compilation.value);
        REQUIRE(bounded.graph);
        CHECK(bounded.stats.samples == reference.sampling.stats.samples);
        CHECK(bounded.stats.boundaryIntervals == reference.sampling.stats.boundaryIntervals);
        CHECK(bounded.graph->metrics().size() == (**reference.compilation.value).metrics().size());
        return std::make_pair(bounded.stats.samples, bounded.stats.boundaryIntervals);
    };
    CHECK(run(false) == run(true));
}

#ifdef DIG_NATIVE_TRANSFERS
TEST_CASE("V2 native transfers follow corridors and enforce resource limits") {
    Rect left{0,0,2,8},leftTop{0,8,2,10},top{2,8,8,10},rightTop{8,8,10,10},right{8,0,10,8};
    left.neighbors[2]=1;leftTop.neighbors[0]=0;leftTop.neighbors[1]=2;
    top.neighbors[3]=1;top.neighbors[1]=3;rightTop.neighbors[3]=2;
    rightTop.neighbors[0]=4;right.neighbors[2]=3;
    auto mesh=singleMesh({left,leftTop,top,rightTop,right},1,{},true);
    BuildInput input;input.navMesh=mesh.get();
    const auto built=buildGraph(input,{2,1,1,1,1000,1000});
    REQUIRE(built.compilation.value);
    const auto& graph=**built.compilation.value;
    const auto base=mesh->getPolyRefBase(static_cast<const dtNavMesh&>(*mesh).getTile(0));
    const Anchor a{0,base,{1,0,1}},b{0,base|4,{9,0,1}};
    NativeTransferProvider provider;
    auto o=nativeLimits();
    REQUIRE(provider.begin(*mesh,graph,0,o)==TransferStatus::Success);
    const auto result=provider.evaluate(0,a,b);
    REQUIRE(result.status==TransferStatus::Success);
    CHECK(result.cost>18); // Direct off-mesh shortcut would cost only 8.
    const auto baseline=provider.stats();
    CHECK(provider.evaluate(0,a,b).cost==result.cost);
    CHECK(provider.stats().cacheHits==1);
    CHECK(provider.stats().queries==baseline.queries);
    CHECK(provider.evaluate(0,a,a).cost==0);
    auto noCache=o;noCache.cacheEntries=1;
    REQUIRE(provider.begin(*mesh,graph,0,noCache)==TransferStatus::Success);
    CHECK(provider.evaluate(0,a,b).status==TransferStatus::Success);
    CHECK(provider.evaluate(0,b,a).status==TransferStatus::Success);
    CHECK(provider.stats().cacheEntries==1);
    CHECK(provider.evaluate(0,a,b).cost==result.cost);
    CHECK(provider.stats().cacheHits==1);
    auto same=findNativeRoute(graph,*mesh,0,a,b,o,{},&provider);
    CHECK(same.route.status==RouteStatus::SameIsland);
    CHECK_FALSE(same.route.value);
    CHECK(provider.stats().cacheEntries==0);
    for(int limit=0;limit<4;++limit) {
        INFO(limit);
        auto capped=o;
        if(limit==0)capped.maxNodes=4;
        if(limit==1)capped.maxCorridor=1;
        if(limit==2)capped.maxStraightPoints=1;
        if(limit==3)capped.maxIterations=1;
        REQUIRE(provider.begin(*mesh,graph,0,capped)==TransferStatus::Success);
        CHECK(provider.evaluate(0,a,b).status==TransferStatus::BudgetExceeded);
    }
    o.maxIterations=baseline.iterations;
    REQUIRE(provider.begin(*mesh,graph,0,o)==TransferStatus::Success);
    CHECK(provider.evaluate(0,a,b).status==TransferStatus::Success);
    o=nativeLimits();o.maxQueries=1;o.cacheEntries=0;
    REQUIRE(provider.begin(*mesh,graph,0,o)==TransferStatus::Success);
    CHECK(provider.evaluate(0,a,b).status==TransferStatus::Success);
    CHECK(provider.evaluate(0,b,a).status==TransferStatus::BudgetExceeded);
    o=nativeLimits();bool cancel=false;o.canceled=[&]{return cancel;};
    REQUIRE(provider.begin(*mesh,graph,0,o)==TransferStatus::Success);
    cancel=true;CHECK(provider.evaluate(0,a,b).status==TransferStatus::Canceled);
    REQUIRE(provider.begin(*mesh,graph,0,nativeLimits())==TransferStatus::Success);
    CHECK(provider.evaluate(0,a,b).cost==result.cost);
    auto wrong=a;wrong.polygon=0;CHECK(provider.checkAnchor(wrong)==TransferStatus::InvalidInput);
    wrong=a;wrong.island=1;CHECK(provider.checkAnchor(wrong)==TransferStatus::InvalidInput);
    wrong=a;wrong.position.y=1;CHECK(provider.checkAnchor(wrong)==TransferStatus::InvalidInput);
    wrong=a;wrong.position.x=std::numeric_limits<float>::quiet_NaN();
    CHECK(provider.checkAnchor(wrong)==TransferStatus::InvalidInput);
    CHECK(provider.begin(*mesh,graph,1,nativeLimits())==TransferStatus::InvalidInput);
    CHECK(provider.evaluate(0,a,b).status==TransferStatus::InvalidInput);
    auto invalid=nativeLimits();invalid.maxNodes=1;
    CHECK(provider.begin(*mesh,graph,0,invalid)==TransferStatus::InvalidInput);
    invalid=nativeLimits();invalid.canceled=[]()->bool{throw std::runtime_error("test");};
    CHECK(provider.begin(*mesh,graph,0,invalid)==TransferStatus::CallbackFailed);
}

TEST_CASE("V2 native filter rejects excluded polygons during search") {
    Rect a{0,0,2,2},b{2,0,4,2},c{4,0,6,2};
    a.neighbors[1]=1;b.neighbors[3]=0;b.neighbors[1]=2;c.neighbors[3]=1;
    auto mesh=singleMesh({a,b,c});
    BuildInput input;input.navMesh=mesh.get();
    const auto sampled=extractAndSample(input,{1,1,1,1,1000,1000});
    REQUIRE(sampled.value);
    auto topology=sampled.value->topology;
    // Trusted producer fixture retains one island while excluding the bridge.
    const auto base=mesh->getPolyRefBase(static_cast<const dtNavMesh&>(*mesh).getTile(0));
    topology.polygons.erase(std::remove_if(topology.polygons.begin(),topology.polygons.end(),
        [&](const PolygonIsland& p){return p.polygon==(base|1);}),topology.polygons.end());
    topology.metrics.clear();
    auto crossings=validateCrossings(topology,{1,1,1,1,1000,1000},{},{});
    REQUIRE(crossings.value);
    auto compiled=compileGraph(*crossings.value);
    REQUIRE(compiled.value);
    NativeTransferProvider provider;
    REQUIRE(provider.begin(*mesh,**compiled.value,0,nativeLimits())==TransferStatus::Success);
    Anchor from{0,base,{1,0,1}},to{0,base|2,{5,0,1}};
    CHECK(provider.evaluate(0,from,to).status==TransferStatus::Blocked);
    CHECK(provider.evaluate(0,from,to).status==TransferStatus::Blocked);
    CHECK(provider.stats().cacheHits==1);
}

TEST_CASE("V2 native detour changes the selected crossing") {
    Rect left{0,0,2,8},leftTop{0,8,2,10},top{2,8,8,10},rightTop{8,8,10,10},right{8,0,10,8};
    left.neighbors[2]=1;leftTop.neighbors[0]=0;leftTop.neighbors[1]=2;
    top.neighbors[3]=1;top.neighbors[1]=3;rightTop.neighbors[3]=2;
    rightTop.neighbors[0]=4;right.neighbors[2]=3;
    auto mesh=singleMesh({left,leftTop,top,rightTop,right,{12,0,14,10}});
    BuildInput input;input.navMesh=mesh.get();
    DiscoveryConfig config{2,30,1,1,1000,1000};
    auto sampled=extractAndSample(input,config);REQUIRE(sampled.value);
    const auto base=mesh->getPolyRefBase(static_cast<const dtNavMesh&>(*mesh).getTile(0));
    Anchor start{0,base,{1,0,1}},end{1,base|5,{13,0,1}};
    const Anchor near{0,base|4,{9,0,1}},detour{0,base,{1,0,7}};
    auto crossings=validateCrossings(sampled.value->topology,config,{{near,end},{detour,end}});
    REQUIRE(crossings.value);auto compiled=compileGraph(*crossings.value);REQUIRE(compiled.value);
    const auto& graph=**compiled.value;
    const auto estimated=findRoute(graph,start,end);
    auto native=findNativeRoute(graph,*mesh,0,start,end,nativeLimits());
    REQUIRE(estimated.value);REQUIRE(native.route.value);
    CHECK(estimated.value->legs.front().from.polygon==near.polygon);
    CHECK(native.route.value->legs.front().from.polygon==detour.polygon);
    CHECK(native.route.value->totalCost==doctest::Approx(6+std::sqrt(180.f)));
    auto canceled=nativeLimits();std::size_t checks=0;
    canceled.canceled=[&]{return ++checks>15;};
    auto failed=findNativeRoute(graph,*mesh,0,start,end,canceled);
    CHECK(failed.route.status==RouteStatus::Canceled);CHECK_FALSE(failed.route.value);
}
#endif
