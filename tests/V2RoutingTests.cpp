#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>
#include <detour_island_graph/v2/Routing.h>
#include <DetourNavMeshBuilder.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
#include <set>
#include <tuple>
#include "V2RouteOracle.h"

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

float distance(Point a, Point b) {
    const double dx = double(b.x) - a.x;
    const double dy = double(b.y) - a.y;
    const double dz = double(b.z) - a.z;
    return float(std::sqrt(dx * dx + dy * dy + dz * dz));
}

std::shared_ptr<const CompiledGraph> buildPair(const Mesh& mesh) {
    BuildInput input;
    input.navMesh = mesh.get();
    const auto result = buildGraph(input, {2, 3, 2, 4, 0, 0});
    REQUIRE(result.status == StageStatus::Success);
    REQUIRE(result.compilation.value);
    return *result.compilation.value;
}

void checkChain(const Route& route, IslandId start, IslandId end) {
    REQUIRE_FALSE(route.legs.empty());
    CHECK(route.legs.front().from.island == start);
    CHECK(route.legs.back().to.island == end);
    for (std::size_t i = 1; i < route.legs.size(); ++i)
        CHECK(route.legs[i].from.island == route.legs[i - 1].to.island);
}
} // namespace

TEST_CASE("V2 routing finds geometric routes with estimated cost") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    auto graph = buildPair(mesh);
    RouteScratch scratch;
    const auto result = findRoute(*graph, 0, 1, {0, 0, 0}, {6, 0, 2}, {}, &scratch);
    CHECK(result.status == RouteStatus::Success);
    REQUIRE(result.value);
    CHECK(result.stats.estimatedCost);
    checkChain(*result.value, 0, 1);
    const auto& leg = result.value->legs.front();
    const float expected = distance({0, 0, 0}, leg.from.position) +
        distance(leg.from.position, leg.to.position) + distance(leg.to.position, {6, 0, 2});
    CHECK(result.value->totalCost == doctest::Approx(expected));

    // Scratch reuse matches a scratch-less query; reverse uses BA traversals.
    const auto fresh = findRoute(*graph, 0, 1, {0, 0, 0}, {6, 0, 2});
    REQUIRE(fresh.value);
    CHECK(fresh.value->legs.size() == result.value->legs.size());
    const auto back = findRoute(*graph, 1, 0, {6, 0, 2}, {0, 0, 0});
    CHECK(back.status == RouteStatus::Success);
    REQUIRE(back.value);
    checkChain(*back.value, 1, 0);
}

TEST_CASE("V2 routing reports same island invalid and blocked queries") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    auto graph = buildPair(mesh);
    CHECK(findRoute(*graph, 0, 0, {0, 0, 0}, {2, 0, 2}).status == RouteStatus::SameIsland);
    CHECK(findRoute(*graph, 0, 9, {0, 0, 0}, {6, 0, 0}).status == RouteStatus::InvalidIsland);
    RouteOptions blocked;
    blocked.crossingFilter = [](const CompiledCrossing&, bool, const RouteCostContext&) {
        return false;
    };
    CHECK(findRoute(*graph, 0, 1, {0, 0, 0}, {6, 0, 0}, blocked).status == RouteStatus::NoPath);

    RouteOptions invalid;
    invalid.crossingCost = [](const CompiledCrossing&, bool, const RouteCostContext&) { return -1.0f; };
    CHECK(findRoute(*graph, 0, 1, {0, 0, 0}, {6, 0, 0}, invalid).status == RouteStatus::NoPath);
}

TEST_CASE("V2 routing custom costs use Dijkstra without hiding estimated components") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    auto graph = buildPair(mesh);
    RouteOptions scaled;
    scaled.transferCost = [](IslandId, const Anchor& a, const Anchor& b) {
        return 2 * distance(a.position, b.position);
    };
    const auto result = findRoute(*graph, 0, 1, {0, 0, 0}, {6, 0, 2}, scaled);
    CHECK(result.status == RouteStatus::Success);
    REQUIRE(result.value);
    CHECK(result.stats.estimatedCost);
    CHECK_FALSE(result.stats.usedAStar);
    const auto& leg = result.value->legs.front();
    const float expected = 2 * distance({0, 0, 0}, leg.from.position) +
        distance(leg.from.position, leg.to.position) + 2 * distance(leg.to.position, {6, 0, 2});
    CHECK(result.value->totalCost == doctest::Approx(expected));
}

TEST_CASE("V2 routing prefers cheap multi-hop over an expensive direct crossing") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}, {0, 4, 2, 6}, {4, 4, 6, 6}});
    BuildInput input;
    input.navMesh = mesh.get();
    const auto built = buildGraph(input, {2, 3, 2, 4, 0, 0});
    REQUIRE(built.status == StageStatus::Success);
    auto graph = *built.compilation.value;
    REQUIRE(graph->crossings().size() >= 2);

    RouteOptions priced;
    priced.crossingCost = [](const CompiledCrossing& crossing, bool, const RouteCostContext&) {
        const bool direct = (crossing.crossing.a.island == 0 && crossing.crossing.b.island == 3) ||
            (crossing.crossing.a.island == 3 && crossing.crossing.b.island == 0);
        return direct ? 100.0f : 1.0f;
    };
    const auto result = findRoute(*graph, 0, 3, {0, 0, 0}, {6, 0, 6}, priced);
    CHECK(result.status == RouteStatus::Success);
    REQUIRE(result.value);
    CHECK(result.stats.estimatedCost);
    REQUIRE(result.value->legs.size() == 2);
    for (const auto& leg : result.value->legs) {
        const bool direct = (leg.from.island == 0 && leg.to.island == 3) ||
            (leg.from.island == 3 && leg.to.island == 0);
        CHECK_FALSE(direct);
    }
    checkChain(*result.value, 0, 3);
}

TEST_CASE("V2 routing budgets cancellation input and callback failures") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    auto graph = buildPair(mesh);

    RouteOptions budget;
    budget.maxQueuedPortals = 1;
    const auto limited = findRoute(*graph, 0, 1, {0, 0, 0}, {6, 0, 0}, budget);
    CHECK(limited.status == RouteStatus::BudgetExceeded);
    CHECK_FALSE(limited.value);

    CHECK(limited.stats.queuedPortals == 1);
    CHECK(limited.stats.examinedTraversals == 2);
    CHECK(limited.stats.transferEvaluations == 2);
    CHECK(limited.stats.crossingEvaluations == 2);

    RouteOptions canceled;
    canceled.canceled = [] { return true; };
    CHECK(findRoute(*graph, 0, 1, {0, 0, 0}, {6, 0, 0}, canceled).status == RouteStatus::Canceled);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK(findRoute(*graph, 0, 1, {nan, 0, 0}, {6, 0, 0}).status == RouteStatus::InvalidInput);

    RouteOptions throwing;
    throwing.crossingFilter = [](const CompiledCrossing&, bool, const RouteCostContext&) -> bool {
        throw std::runtime_error("filter");
    };
    CHECK(findRoute(*graph, 0, 1, {0, 0, 0}, {6, 0, 0}, throwing).status ==
        RouteStatus::CallbackFailed);
}

TEST_CASE("V2 routing stops at the proven bound before exhausting expansion budget") {
    TopologyArtifact topology;
    topology.islandCount = 3;
    topology.polygons = {{1, 0}, {2, 1}, {3, 2}};
    const auto validated = validateCrossings(topology, {1, 20, 0, 0}, {
        {{0, 1, {}}, {1, 2, {1, 0, 0}}},
        {{0, 1, {}}, {2, 3, {10, 0, 0}}}});
    REQUIRE(validated.value);
    const auto compiled = compileGraph(*validated.value);
    REQUIRE(compiled.value);
    RouteOptions options;
    options.maxExpandedPortals = 1;
    SUBCASE("geometric A star") {}
    SUBCASE("custom Dijkstra") {
        options.transferCost = [](IslandId, const Anchor& a, const Anchor& b) {
            return distance(a.position, b.position);
        };
    }
    const auto result = findRoute(**compiled.value, 0, 1, {}, {1, 0, 0}, options);
    REQUIRE(result.value);
    CHECK(result.stats.expandedPortals == 1);
    CHECK(result.value->totalCost == 1);
}

TEST_CASE("V2 routing waits for competing arrival when native finish cost is expensive") {
    TopologyArtifact topology;
    topology.islandCount = 2;
    topology.polygons = {{1, 0}, {2, 1}};
    const auto validated = validateCrossings(topology, {1, 20, 0, 0}, {
        {{0, 1, {}}, {1, 2, {1, 0, 0}}},
        {{0, 1, {}}, {1, 2, {2, 0, 0}}}});
    REQUIRE(validated.value);
    const auto compiled = compileGraph(*validated.value);
    REQUIRE(compiled.value);
    RouteOptions options;
    options.transferCost = [](IslandId island, const Anchor& from, const Anchor&) {
        return island == 1 && from.position.x == 1 ? 100.0f : 0.0f;
    };
    const auto result = findRoute(**compiled.value, 0, 1, {}, {3, 0, 0}, options);
    REQUIRE(result.value);
    CHECK(result.value->totalCost == 2);
    CHECK(result.value->legs.back().to.position.x == 2);
    options.maxExpandedPortals = 1;
    const auto limited = findRoute(**compiled.value, 0, 1, {}, {3, 0, 0}, options);
    CHECK(limited.status == RouteStatus::BudgetExceeded);
    CHECK_FALSE(limited.value);
}

TEST_CASE("V2 routing tracks transfer and crossing accuracy independently") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    auto graph = buildPair(mesh);
    RouteOptions options;
    options.transferCost = [](IslandId, const Anchor& a, const Anchor& b) { return distance(a.position, b.position); };
    options.crossingCost = [](const CompiledCrossing&, bool, const RouteCostContext&) { return 1.0f; };
    bool estimated = true;
    SUBCASE("custom defaults remain estimates") {}
    SUBCASE("measured transfers only") { options.transferCostEstimated = false; }
    SUBCASE("both explicitly measured") {
        options.transferCostEstimated = options.crossingCostEstimated = false;
        estimated = false;
    }
    SUBCASE("missing callback cannot mark Euclidean measured") {
        options.transferCost = {};
        options.transferCostEstimated = options.crossingCostEstimated = false;
    }
    const auto result = findRoute(*graph, 0, 1, {}, {6, 0, 0}, options);
    REQUIRE(result.value);
    CHECK(result.stats.estimatedCost == estimated);
    CHECK(result.stats.estimatedTransferCost == (!options.transferCost || options.transferCostEstimated));
    CHECK(result.stats.estimatedCrossingCost == options.crossingCostEstimated);
    CHECK_FALSE(result.stats.usedAStar);
}

TEST_CASE("V2 routing drains stale entries without charging expansion budget") {
    TopologyArtifact topology;
    topology.islandCount = 4;
    topology.polygons = {{1, 0}, {2, 1}, {3, 2}, {4, 3}};
    const auto validated = validateCrossings(topology, {1, 20, 0, 0}, {
        {{0, 1, {}}, {1, 2, {1, 0, 0}}},
        {{0, 1, {}}, {1, 2, {2, 0, 0}}},
        {{1, 2, {3, 0, 0}}, {2, 3, {4, 0, 0}}}});
    REQUIRE(validated.value);
    const auto compiled = compileGraph(*validated.value);
    REQUIRE(compiled.value);
    RouteOptions options;
    options.maxExpandedPortals = 3;
    options.crossingFilter = [](const CompiledCrossing&, bool reverse, const RouteCostContext&) { return !reverse; };
    options.transferCost = [](IslandId island, const Anchor& from, const Anchor&) {
        return island == 1 && from.position.x == 1 ? 10.0f : 0.0f;
    };
    const auto result = findRoute(**compiled.value, 0, 3, {}, {}, options);
    CHECK(result.status == RouteStatus::NoPath);
    CHECK(result.stats.expandedPortals == 3);
    CHECK(result.stats.queuedPortals == 4);
    CHECK(result.stats.heapPops == 4);
    CHECK(result.stats.staleHeapPops == 1);
}

TEST_CASE("V2 routing observes cancellation triggered by final transfer") {
    auto mesh = makeMesh({{0, 0, 2, 2}, {4, 0, 6, 2}});
    auto graph = buildPair(mesh);
    bool canceled = false;
    RouteOptions options;
    options.canceled = [&] { return canceled; };
    options.transferCost = [&](IslandId island, const Anchor& a, const Anchor& b) {
        if (island == 1 && b.polygon == 0) canceled = true;
        return distance(a.position, b.position);
    };
    const auto result = findRoute(*graph, 0, 1, {}, {6, 0, 0}, options);
    CHECK(result.status == RouteStatus::Canceled);
    CHECK_FALSE(result.value);
}

TEST_CASE("V2 route counters preserve attempts and scratch survives aborts") {
    TopologyArtifact topology;topology.islandCount=2;topology.polygons={{1,0},{2,1}};
    auto validated=validateCrossings(topology,{1,20,0,0},{{{0,1,{}},{1,2,{1,0,0}}}});
    REQUIRE(validated.value);auto compiled=compileGraph(*validated.value);REQUIRE(compiled.value);
    const auto& graph=**compiled.value;RouteScratch scratch;RouteOptions options;
    SUBCASE("filtered") {options.crossingFilter=[](const CompiledCrossing&,bool,const RouteCostContext&){return false;};}
    SUBCASE("throwing transfer") {options.transferCost=[](IslandId,const Anchor&,const Anchor&)->float{throw std::runtime_error("cost");};}
    SUBCASE("canceled after transfer") {
        bool cancel=false;options.canceled=[&]{return cancel;};
        options.transferCost=[&](IslandId,const Anchor&,const Anchor&){cancel=true;return 0.f;};
        auto r=findRoute(graph,0,1,{}, {1,0,0},options,&scratch);
        CHECK(r.status==RouteStatus::Canceled);CHECK(r.stats.transferEvaluations==1);CHECK(r.stats.crossingEvaluations==0);
        options={};
    }
    auto r=findRoute(graph,0,1,{}, {1,0,0},options,&scratch);
    CHECK(r.stats.examinedTraversals==1);
    if(options.crossingFilter) {CHECK(r.stats.transferEvaluations==0);CHECK(r.stats.heapPops==0);}
    else if(options.transferCost) {CHECK(r.status==RouteStatus::CallbackFailed);CHECK(r.stats.transferEvaluations==1);CHECK(r.stats.crossingEvaluations==0);}
    else {CHECK(r.stats.transferEvaluations==2);CHECK(r.stats.crossingEvaluations==1);CHECK(r.stats.heapPops==1);}
    const auto reused=findRoute(graph,0,1,{}, {1,0,0},{},&scratch),fresh=findRoute(graph,0,1,{}, {1,0,0});
    REQUIRE(reused.value);REQUIRE(fresh.value);CHECK(reused.value->totalCost==fresh.value->totalCost);
    CHECK(reused.stats.examinedTraversals==fresh.stats.examinedTraversals);
    CHECK(reused.stats.transferEvaluations==fresh.stats.transferEvaluations);
    CHECK(reused.stats.crossingEvaluations==fresh.stats.crossingEvaluations);
    CHECK(reused.stats.heapPops==fresh.stats.heapPops);CHECK(reused.stats.staleHeapPops==fresh.stats.staleHeapPops);
    CHECK(reused.stats.expandedPortals==fresh.stats.expandedPortals);CHECK(reused.stats.queuedPortals==fresh.stats.queuedPortals);
}

TEST_CASE("V2 independent minimum cost oracle covers custom and directed routes") {
    TopologyArtifact topology;topology.islandCount=4;topology.polygons={{1,0},{2,1},{3,2},{4,3}};
    auto validated=validateCrossings(topology,{1,20,0,0},{
        {{0,1,{}},{1,2,{1,0,0}}},{{0,1,{}},{1,2,{2,0,0}}},{{1,2,{3,0,0}},{2,3,{4,0,0}}}});
    REQUIRE(validated.value);auto compiled=compileGraph(*validated.value);REQUIRE(compiled.value);
    const auto& graph=**compiled.value;RouteOptions o;
    o.crossingFilter=[](const CompiledCrossing&,bool reverse,const RouteCostContext&){return !reverse;};
    o.transferCost=[](IslandId,const Anchor&,const Anchor&){return 0.f;};
    o.crossingCost=[](const CompiledCrossing&,bool,const RouteCostContext&){return 1.f;};
    double expected=2;
    SUBCASE("equal competing routes") {}
    SUBCASE("zero costs") {o.crossingCost=[](const CompiledCrossing&,bool,const RouteCostContext&){return 0.f;};expected=0;}
    SUBCASE("expensive early arrival") {o.transferCost=[](IslandId i,const Anchor& a,const Anchor&){return i==1&&a.position.x==1?100.f:0.f;};}
    SUBCASE("blocked transfers") {o.transferCost=[](IslandId i,const Anchor&,const Anchor&){return i==1?std::numeric_limits<float>::infinity():0.f;};expected=std::numeric_limits<double>::infinity();}
    const auto minimum=fixture_oracle::minimumCost(graph,0,2,{}, {4,0,0},o);
    CHECK(minimum==expected);const auto route=findRoute(graph,0,2,{}, {4,0,0},o);
    if(std::isfinite(expected)){REQUIRE(route.value);CHECK(route.value->totalCost==doctest::Approx(expected));}
    else CHECK(route.status==RouteStatus::NoPath);
    CHECK_FALSE(std::isfinite(fixture_oracle::minimumCost(graph,2,0,{4,0,0},{},o)));
    CHECK_FALSE(std::isfinite(fixture_oracle::minimumCost(graph,0,3,{},{},o)));
}

namespace {
std::shared_ptr<const CompiledGraph> dominanceGraph(bool distinctPolygon=false) {
    TopologyArtifact topology;
    topology.islandCount=4;topology.polygons={{1,0},{2,1},{3,2},{4,3},{5,1}};
    auto validated=validateCrossings(topology,{1,50,0,0},{
        {{0,1,{0,0,0}},{1,2,{2,0,0}}},
        {{0,1,{10,0,0}},{1,distinctPolygon?dtPolyRef(5):dtPolyRef(2),{2,0,0}}},
        {{1,2,{2,0,0}},{2,3,{3,0,0}}}});
    REQUIRE(validated.value);auto compiled=compileGraph(*validated.value);REQUIRE(compiled.value);
    return *compiled.value;
}
}

TEST_CASE("V2 arrival dominance suppresses equal and expensive arrivals before budget") {
    auto graph=dominanceGraph();RouteScratch scratch;
    auto optimized=findRoute(*graph,0,3,{},{},{},&scratch);
    RouteOptions off;off.enableArrivalDominance=false;
    auto reference=findRoute(*graph,0,3,{},{},off);
    CHECK(optimized.status==RouteStatus::NoPath);
    CHECK(reference.status==optimized.status);
    CHECK(optimized.stats.usedArrivalDominance);
    CHECK(optimized.stats.dominatedArrivals>0);
    CHECK(optimized.stats.examinedTraversals<reference.stats.examinedTraversals);
    CHECK(optimized.stats.expandedPortals<=optimized.stats.arrivalGroups);
    CHECK(optimized.stats.heapPops==optimized.stats.expandedPortals+
        optimized.stats.staleHeapPops+optimized.stats.dominatedArrivals);
    RouteOptions cap;cap.maxExpandedPortals=optimized.stats.expandedPortals;
    CHECK(findRoute(*graph,0,3,{},{},cap).status==RouteStatus::NoPath);
    --cap.maxExpandedPortals;
    auto limited=findRoute(*graph,0,3,{},{},cap,&scratch);
    CHECK(limited.status==RouteStatus::BudgetExceeded);CHECK_FALSE(limited.value);
    auto reused=findRoute(*graph,0,3,{},{},{},&scratch);
    CHECK(fixture_oracle::sameWork(reused.stats,optimized.stats));
    // The expensive incoming traversal is initialized too, but the cheap arrival wins.
    auto route=findRoute(*graph,0,2,{}, {3,0,0});REQUIRE(route.value);
    CHECK(route.value->totalCost==doctest::Approx(3));
    CHECK(fixture_oracle::minimumCost(*graph,0,2,{}, {3,0,0})==doctest::Approx(3));
}

TEST_CASE("V2 typed transfers preserve anchors and propagate fatal outcomes") {
    auto graph=dominanceGraph();RouteScratch scratch;
    const Anchor start{0,1,{0,0,0}},end{2,3,{3,0,0}};
    RouteOptions options;options.transferCostEstimated=false;
    options.transferEvaluator=[](IslandId island,const Anchor& a,const Anchor& b) {
        CHECK(a.polygon!=0);CHECK(b.polygon!=0);CHECK(a.island==island);CHECK(b.island==island);
        return TransferResult{TransferStatus::Success,distance(a.position,b.position)};
    };
    const auto good=findRoute(*graph,start,end,options,&scratch);
    REQUIRE(good.value);CHECK(good.value->totalCost==3);
    CHECK_FALSE(good.stats.usedAStar);CHECK_FALSE(good.stats.usedArrivalDominance);
    CHECK_FALSE(good.stats.estimatedTransferCost);CHECK(good.stats.estimatedCrossingCost);
    for(const auto pair:std::vector<std::pair<TransferStatus,RouteStatus>>{
        {TransferStatus::Blocked,RouteStatus::NoPath},{TransferStatus::InvalidInput,RouteStatus::InvalidInput},
        {TransferStatus::BudgetExceeded,RouteStatus::BudgetExceeded},{TransferStatus::Canceled,RouteStatus::Canceled},
        {TransferStatus::OutOfMemory,RouteStatus::OutOfMemory},{TransferStatus::CallbackFailed,RouteStatus::CallbackFailed}}) {
        auto failed=options;failed.transferEvaluator=[&](IslandId,const Anchor&,const Anchor&){return TransferResult{pair.first};};
        const auto r=findRoute(*graph,start,end,failed,&scratch);
        CHECK(r.status==pair.second);CHECK_FALSE(r.value);
    }
    auto invalid=options;invalid.transferCost=[](IslandId,const Anchor&,const Anchor&){return 0.f;};
    CHECK(findRoute(*graph,start,end,invalid).status==RouteStatus::InvalidInput);
    invalid=options;invalid.transferEvaluator=[](IslandId,const Anchor&,const Anchor&)->TransferResult{throw std::runtime_error("test");};
    CHECK(findRoute(*graph,start,end,invalid).status==RouteStatus::CallbackFailed);
    invalid.transferEvaluator=[](IslandId,const Anchor&,const Anchor&){return TransferResult{TransferStatus::Success,-1};};
    CHECK(findRoute(*graph,start,end,invalid).status==RouteStatus::InvalidInput);
    auto wrong=start;wrong.polygon=0;
    CHECK(findRoute(*graph,wrong,end,options).status==RouteStatus::InvalidInput);
    CHECK(findRoute(*graph,start,end,options,&scratch).value->totalCost==good.value->totalCost);
}

TEST_CASE("V2 arrival groups preserve exact polygon island and position identity") {
    auto graph=dominanceGraph(true);
    using Key=std::tuple<IslandId,dtPolyRef,float,float,float>;
    std::set<Key> expected;
    for(const auto& c:graph->crossings()) {
        auto add=[&](const Anchor& a){expected.emplace(a.island,a.polygon,a.position.x,a.position.y,a.position.z);};
        if(c.traversableAB)add(c.crossing.b);if(c.traversableBA)add(c.crossing.a);
    }
    auto route=findRoute(*graph,0,3,{},{});
    CHECK(route.stats.arrivalGroups==expected.size());
    CHECK(route.stats.arrivalGroups==5);
    TopologyArtifact topology;topology.islandCount=3;topology.polygons={{1,0},{2,1},{3,2}};
    auto validated=validateCrossings(topology,{1,1,0,0},{
        {{0,1,{}},{1,2,{}}},{{1,2,{}},{2,3,{}}}});
    REQUIRE(validated.value);auto compiled=compileGraph(*validated.value);REQUIRE(compiled.value);
    RouteScratch scratch;
    findRoute(*graph,0,3,{},{},{},&scratch);
    auto zero=findRoute(**compiled.value,0,2,{},{},{},&scratch);
    REQUIRE(zero.value);CHECK(zero.value->totalCost==0);CHECK(zero.stats.arrivalGroups==3);
    CHECK(fixture_oracle::sameWork(zero.stats,findRoute(**compiled.value,0,2,{},{}).stats));
}

TEST_CASE("V2 arrival dominance preserves each custom callback execution") {
    auto graph=dominanceGraph();RouteOptions options;std::size_t calls=0;
    SUBCASE("transfer") {options.transferCost=[&](IslandId,const Anchor& a,const Anchor& b){++calls;return distance(a.position,b.position);};}
    SUBCASE("crossing") {options.crossingCost=[&](const CompiledCrossing& c,bool,const RouteCostContext&){++calls;return distance(c.crossing.a.position,c.crossing.b.position);};}
    SUBCASE("filter") {options.crossingFilter=[&](const CompiledCrossing&,bool reverse,const RouteCostContext&){++calls;return !reverse;};}
    const auto enabled=findRoute(*graph,0,2,{}, {3,0,0},options);
    const auto enabledCalls=calls;calls=0;options.enableArrivalDominance=false;
    const auto disabled=findRoute(*graph,0,2,{}, {3,0,0},options);
    CHECK_FALSE(enabled.stats.usedArrivalDominance);CHECK(enabled.stats.arrivalGroups==0);
    CHECK(calls==enabledCalls);CHECK(calls>0);
    CHECK(fixture_oracle::sameWork(enabled.stats,disabled.stats));fixture_oracle::equivalent(enabled,disabled);
}

TEST_CASE("V2 arrival preparation cancellation leaves reusable scratch") {
    auto graph=dominanceGraph();RouteScratch scratch;RouteOptions options;
    std::size_t checks=0;
    SUBCASE("collect") {options.canceled=[&]{return ++checks==3;};}
    SUBCASE("sort") {options.canceled=[&]{return ++checks==graph->traversals().size()+2;};}
    SUBCASE("group") {options.canceled=[&]{return !scratch.arrivalBest.empty();};}
    SUBCASE("search") {options.canceled=[&]{return !scratch.heap.empty();};}
    auto canceled=findRoute(*graph,0,3,{},{},options,&scratch);
    CHECK(canceled.status==RouteStatus::Canceled);CHECK_FALSE(canceled.value);
    CHECK(canceled.stats.usedArrivalDominance);
    auto reused=findRoute(*graph,0,3,{},{},{},&scratch);
    CHECK(fixture_oracle::sameWork(reused.stats,findRoute(*graph,0,3,{},{}).stats));
    RouteOptions off;off.enableArrivalDominance=false;
    auto fallback=findRoute(*graph,0,3,{},{},off,&scratch);
    CHECK(fixture_oracle::sameWork(fallback.stats,findRoute(*graph,0,3,{},{},off).stats));
}

TEST_CASE("V2 arrival preparation rejects malformed anchor keys") {
    auto graph=std::make_shared<CompiledGraph>(*dominanceGraph());
    auto& crossings=const_cast<BuildVector<CompiledCrossing>&>(graph->crossings());
    SUBCASE("nonfinite") {crossings[0].crossing.b.position.x=std::numeric_limits<float>::quiet_NaN();}
    SUBCASE("wrong owner") {crossings[0].crossing.b.polygon=1;}
    SUBCASE("missing polygon") {crossings[0].crossing.b.polygon=999;}
    const auto result=findRoute(*graph,0,3,{},{});
    CHECK(result.status==RouteStatus::InvalidInput);CHECK_FALSE(result.value);
}
