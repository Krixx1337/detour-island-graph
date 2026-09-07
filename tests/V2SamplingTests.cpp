#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>
#include <DetourNavMeshBuilder.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <tuple>
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
