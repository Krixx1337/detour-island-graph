#include <doctest/doctest.h>
#include <detour_island_graph/v2/Health.h>
#include <limits>
#include "V2RouteOracle.h"

namespace {
using namespace detour_island_graph::v2;
CrossingArtifact artifact(std::size_t n, const std::vector<std::pair<IslandId,IslandId>>& edges) {
    CrossingArtifact a; a.topology.islandCount=n; a.validatorSupplied=true;
    a.discovery={1,100,100,100,0,0};
    for (IslandId i=0;i<n;++i) a.topology.polygons.push_back({dtPolyRef(i+1),i});
    for (auto e:edges) {
        bool reverse=e.first>e.second;
        auto lo=std::min(e.first,e.second),hi=std::max(e.first,e.second);
        Crossing c; c.a={lo,dtPolyRef(lo+1),{float(lo),0,0}}; c.b={hi,dtPolyRef(hi+1),{float(hi),0,0}};
        c.ab={true,true,{reverse?ValidationState::Invalid:ValidationState::Valid,0}};
        c.ba={true,true,{reverse?ValidationState::Valid:ValidationState::Invalid,0}};
        a.crossings.push_back(c);
    }
    return a;
}
std::shared_ptr<const CompiledGraph> compile(const CrossingArtifact& a) {
    auto r=compileGraph(a,{CompilePolicy::ValidatedOnly,{}}); REQUIRE(r.status==StageStatus::Success); return *r.value;
}
}
TEST_CASE("V2 health measures directed components and optional metrics") {
    auto a=artifact(5,{{0,1},{1,2},{2,0},{2,3}});
    auto g=compile(a); auto r=analyzeGraphHealth(*g);
    REQUIRE(r.status==StageStatus::Success); REQUIRE(r.value); const auto& h=*r.value;
    CHECK(h.healthy()); CHECK(h.components.size()==3); CHECK(h.isolated==1);
    CHECK(h.referenceIsland==0); CHECK(h.forward.islands==4); CHECK(h.reverse.islands==3); CHECK(h.mutual.islands==3);
    CHECK_FALSE(h.included.surfaceArea); CHECK(h.included.polygons==5);
    // Independent transitive closure verifies SCC membership and reference reachability.
    bool reachable[5][5]{};
    for (int i=0;i<5;++i) reachable[i][i]=true;
    for (auto e:std::vector<std::pair<int,int>>{{0,1},{1,2},{2,0},{2,3}}) reachable[e.first][e.second]=true;
    for(int k=0;k<5;++k) for(int i=0;i<5;++i) for(int j=0;j<5;++j) reachable[i][j]|=reachable[i][k]&&reachable[k][j];
    for(int i=0;i<5;++i) {
        CHECK(h.islands[i].forward==reachable[0][i]); CHECK(h.islands[i].reverse==reachable[i][0]);
        for(int j=0;j<5;++j) CHECK((h.islands[i].component==h.islands[j].component)==(reachable[i][j]&&reachable[j][i]));
    }
    a.topology.metrics.resize(5);
    for(auto& m:a.topology.metrics) {m.polygonCount=1;m.surfaceArea=2;}
    r=analyzeGraphHealth(*compile(a)); REQUIRE(r.value); CHECK(r.value->included.surfaceArea==10);
}
TEST_CASE("V2 health handles empty domains and parallel directions") {
    auto empty=analyzeGraphHealth(*compile(artifact(0,{}))); REQUIRE(empty.value); CHECK(empty.value->healthy()); CHECK_FALSE(empty.value->referenceIsland);
    auto a=artifact(4,{{0,1}}); a.crossings[0].ba.validation.state=ValidationState::Valid;
    auto duplicate=a.crossings[0]; duplicate.a.position.z=1; a.crossings.push_back(duplicate);
    a.topology.customDomainPolicy=true; a.topology.domain.resize(4);
    a.topology.domain[2].state=DomainState::Excluded; a.topology.domain[3].state=DomainState::Unexplored;
    auto r=analyzeGraphHealth(*compile(a)); REQUIRE(r.value); CHECK(r.value->healthy());
    CHECK(r.value->included.islands==2); CHECK(r.value->excluded==1); CHECK(r.value->unexplored==1);
    CHECK(r.value->bothDirections==2); CHECK(r.value->directedTraversals==4); CHECK(r.value->islands[0].outgoingNeighbors==1);
}
TEST_CASE("V2 health traverses long one-way chains iteratively") {
    std::vector<std::pair<IslandId,IslandId>> edges;
    for(IslandId i=0;i<4095;++i)edges.emplace_back(i,i+1);
    auto h=analyzeGraphHealth(*compile(artifact(4096,edges)));
    REQUIRE(h.value);CHECK(h.value->healthy());CHECK(h.value->components.size()==4096);
    CHECK(h.value->forward.islands==4096);CHECK(h.value->reverse.islands==1);CHECK(h.value->isolated==0);
}
TEST_CASE("V2 health bounds analysis and handles cancellation") {
    auto g=compile(artifact(3,{{0,1},{1,2}})); HealthOptions o;
    SUBCASE("islands") {o.maxIslands=1;}
    SUBCASE("crossings") {o.maxCrossings=1;}
    SUBCASE("traversals") {o.maxTraversals=1;}
    SUBCASE("polygons") {o.maxPolygons=1;}
    SUBCASE("zero limit") {o.maxIslands=0; CHECK(analyzeGraphHealth(*g,o).status==StageStatus::InvalidInput);return;}
    SUBCASE("cancel") {o.canceled=[] {return true;}; CHECK(analyzeGraphHealth(*g,o).status==StageStatus::Canceled);return;}
    SUBCASE("late cancel") {int calls=0;o.canceled=[&]{return ++calls>10;};auto r=analyzeGraphHealth(*g,o);CHECK(r.status==StageStatus::Canceled);CHECK_FALSE(r.value);return;}
    SUBCASE("throw") {o.canceled=[]()->bool{throw 1;};CHECK(analyzeGraphHealth(*g,o).status==StageStatus::CallbackFailed);return;}
    auto r=analyzeGraphHealth(*g,o); CHECK(r.status==StageStatus::BudgetExceeded);CHECK_FALSE(r.value);
}
TEST_CASE("V2 health detects corrupt graph views without exposing mutation API") {
    // Copy is non-const. Mutate only test-owned storage through existing const views.
    auto source=compile(artifact(3,{{0,1},{1,2}})); CompiledGraph broken=*source;
    SUBCASE("offset") {const_cast<BuildVector<std::size_t>&>(broken.offsets())[1]=999;}
    SUBCASE("traversal") {const_cast<BuildVector<Traversal>&>(broken.traversals())[0].crossing=999;}
    SUBCASE("direction") {const_cast<BuildVector<CompiledCrossing>&>(broken.crossings())[0].traversableBA=true;}
    SUBCASE("anchor") {const_cast<BuildVector<CompiledCrossing>&>(broken.crossings())[0].crossing.a.position.x=std::numeric_limits<float>::quiet_NaN();}
    SUBCASE("duplicate") {auto& c=const_cast<BuildVector<CompiledCrossing>&>(broken.crossings());c[1]=c[0];}
    SUBCASE("metrics") {auto& m=const_cast<BuildVector<IslandMetrics>&>(broken.metrics());m.resize(3);m[0].surfaceArea=-1;}
    SUBCASE("ownership") {const_cast<BuildUnorderedMap<dtPolyRef,IslandId>&>(broken.polygonIslands())[1]=99;}
    SUBCASE("eligibility") {const_cast<BuildVector<CompiledCrossing>&>(broken.crossings())[0].crossing.a.position.z=1000;}
    HealthOptions o;o.maxIssueExamples=1;
    auto r=analyzeGraphHealth(broken,o);REQUIRE(r.value);CHECK_FALSE(r.value->healthy());CHECK(r.value->examples.size()==1);
}
TEST_CASE("V2 geometric health excludes known invalid directions") {
    auto a=artifact(2,{{0,1}});
    auto g=compileGraph(a,{CompilePolicy::GeometricOnly,{}});REQUIRE(g.value);
    HealthOptions o;o.maxIssueExamples=0;
    auto h=analyzeGraphHealth(**g.value,o);REQUIRE(h.value);CHECK(h.value->healthy());CHECK(h.value->directedTraversals==1);
}
TEST_CASE("V2 compiler rejects malformed producer health inputs") {
    auto a=artifact(2,{{0,1}});
    SUBCASE("ownership") {a.crossings[0].a.polygon=99;}
    SUBCASE("nonfinite") {a.crossings[0].a.position.x=std::numeric_limits<float>::infinity();}
    SUBCASE("self") {a.crossings[0].b=a.crossings[0].a;}
    CHECK(compileGraph(a,{CompilePolicy::ValidatedOnly,{}}).status==StageStatus::InvalidInput);
}
TEST_CASE("Independent route checker detects broken legs and costs") {
    auto g=compile(artifact(2,{{0,1}}));
    auto result=findRoute(*g,0,1,{0,0,0},{1,0,0});REQUIRE(result.value);
    auto route=*result.value;
    CHECK_NOTHROW(fixture_oracle::checkRoute(*g,0,1,{0,0,0},{1,0,0},route));
    SUBCASE("cost") {route.totalCost+=1;}
    SUBCASE("direction") {route.legs[0].reverse=true;}
    SUBCASE("anchor") {route.legs[0].to.position.z+=1;}
    SUBCASE("index") {route.legs[0].crossing=100;}
    CHECK_THROWS(fixture_oracle::checkRoute(*g,0,1,{0,0,0},{1,0,0},route));
}
