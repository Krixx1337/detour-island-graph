#pragma once
#include <detour_island_graph/v2/NativeTransfers.h>
#include "V2RouteOracle.h"
#include <DetourNavMeshQuery.h>
#include <map>
#include <string>
#include <tuple>

namespace native_fixture {
using namespace detour_island_graph::v2;
using fixture_oracle::require;
inline NativeTransferOptions options() {
    // Coarse boundary versus detail projection: Pandora <=.201, Steelribs <=.551.
    // This is a fixture navigation-model allowance, not execution calibration.
    return {1000000,10000000,4096,4096,8192,100000,0.60f,{}};
}
struct Outcome {
    IslandId from,to;
    NativeRouteResult result;
    double milliseconds = 0;
};
// Independent unsliced Detour evaluation. The production provider, cache,
// sliced search and status translation are not used by this oracle.
inline float referenceCost(const CompiledGraph& graph,const dtNavMesh& mesh,Anchor a,Anchor b) {
    struct Filter : dtQueryFilter {
        const CompiledGraph& graph; IslandId island;
        Filter(const CompiledGraph& g,IslandId i):graph(g),island(i){}
        bool passFilter(dtPolyRef ref,const dtMeshTile*,const dtPoly* p) const override {
            const auto it=graph.polygonIslands().find(ref);
            return p->getType()==DT_POLYTYPE_GROUND && it!=graph.polygonIslands().end() && it->second==island;
        }
    } filter(graph,a.island);
    require(a.island==b.island,"Oracle island mismatch");
    dtNavMeshQuery q;require(dtStatusSucceed(q.init(&mesh,4096)),"Oracle query init failed");
    float rawA[]{a.position.x,a.position.y,a.position.z},rawB[]{b.position.x,b.position.y,b.position.z},pa[3],pb[3];
    require(dtStatusSucceed(q.closestPointOnPoly(a.polygon,rawA,pa,nullptr)) &&
        dtStatusSucceed(q.closestPointOnPoly(b.polygon,rawB,pb,nullptr)),"Oracle projection failed");
    dtPolyRef path[4096];int count=0;
    auto s=q.findPath(a.polygon,b.polygon,pa,pb,&filter,path,&count,4096);
    require(dtStatusSucceed(s) && !(s&(DT_OUT_OF_NODES|DT_BUFFER_TOO_SMALL)),"Oracle path resource failure");
    if(s&DT_PARTIAL_RESULT) return std::numeric_limits<float>::infinity();
    require(count>0 && path[0]==a.polygon && path[count-1]==b.polygon,"Oracle corridor incomplete");
    for(int i=0;i<count;++i) {
        const dtMeshTile* tile=nullptr;const dtPoly* poly=nullptr;
        require(dtStatusSucceed(mesh.getTileAndPolyByRef(path[i],&tile,&poly)) && filter.passFilter(path[i],tile,poly),"Oracle corridor ownership failure");
    }
    float points[8192*3];int n=0;
    s=q.findStraightPath(pa,pb,path,count,points,nullptr,nullptr,&n,8192,DT_STRAIGHTPATH_ALL_CROSSINGS);
    require(dtStatusSucceed(s) && !(s&(DT_PARTIAL_RESULT|DT_BUFFER_TOO_SMALL)) && n>0,"Oracle corners incomplete");
    double cost=0;
    for(int i=1;i<n;++i) cost+=fixture_oracle::distance(
        {points[(i-1)*3],points[(i-1)*3+1],points[(i-1)*3+2]},
        {points[i*3],points[i*3+1],points[i*3+2]});
    return float(cost);
}
inline std::vector<Outcome> check(const CompiledGraph& graph, const dtNavMesh& mesh) {
    dtNavMeshQuery audit; require(dtStatusSucceed(audit.init(&mesh,32)),"Audit init failed");
    double maxProjection=0;
    for(const auto& c:graph.crossings()) for(const auto& a:{c.crossing.a,c.crossing.b}) {
        float p[]{a.position.x,a.position.y,a.position.z},q[3];
        require(dtStatusSucceed(audit.closestPointOnPoly(a.polygon,p,q,nullptr)),"Audit projection failed");
        maxProjection=std::max(maxProjection,fixture_oracle::distance(a.position,{q[0],q[1],q[2]}));
    }
    require(maxProjection<=options().projectionTolerance,("Projection displacement="+std::to_string(maxProjection)).c_str());
    std::map<IslandId,Anchor> endpoints;
    for (const auto& entry : graph.polygonIslands()) {
        if (!graph.includes(entry.second)) continue;
        auto found = endpoints.find(entry.second);
        if (found != endpoints.end() && found->second.polygon < entry.first) continue;
        const dtMeshTile* tile=nullptr; const dtPoly* poly=nullptr;
        require(dtStatusSucceed(mesh.getTileAndPolyByRef(entry.first,&tile,&poly)),"Native endpoint polygon missing");
        Point p{};
        for (int i=0;i<poly->vertCount;++i) {
            const auto* v=&tile->verts[poly->verts[i]*3];
            p.x+=v[0]/poly->vertCount;p.y+=v[1]/poly->vertCount;p.z+=v[2]/poly->vertCount;
        }
        dtNavMeshQuery query; require(dtStatusSucceed(query.init(&mesh,32)),"Endpoint query init failed");
        const float raw[]{p.x,p.y,p.z};float projected[3];
        require(dtStatusSucceed(query.closestPointOnPoly(entry.first,raw,projected,nullptr)),"Endpoint projection failed");
        endpoints[entry.second]={entry.second,entry.first,{projected[0],projected[1],projected[2]}};
    }
    NativeTransferProvider provider, reference;
    RouteScratch scratch;
    std::vector<Outcome> outcomes;
    for (const auto& a:endpoints) for(const auto& b:endpoints) {
        auto settings=options(); RouteOptions routing;
        routing.maxExpandedPortals=1000000;routing.maxQueuedPortals=2000000;
        const auto started=std::chrono::steady_clock::now();
        auto result=findNativeRoute(graph,mesh,graph.identity().mesh,a.second,b.second,settings,routing,&provider,&scratch);
        const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
        settings.cacheEntries=0;
        auto uncached=findNativeRoute(graph,mesh,graph.identity().mesh,a.second,b.second,settings,routing,&reference);
        fixture_oracle::equivalent(result.route,uncached.route);
        if (result.route.status!=RouteStatus::Success && result.route.status!=RouteStatus::NoPath &&
            result.route.status!=RouteStatus::SameIsland) throw std::runtime_error("Native fixture query failed " +
                std::to_string(a.first)+" -> "+std::to_string(b.first)+" status="+std::to_string(int(result.route.status))+
                " queries="+std::to_string(result.transfers.queries));
        if (a.first!=b.first) {
            require(!result.route.stats.estimatedTransferCost && result.route.stats.estimatedCrossingCost &&
                !result.route.stats.usedAStar && !result.route.stats.usedArrivalDominance,"Native provenance mismatch");
            using Key=std::tuple<IslandId,dtPolyRef,float,float,float,dtPolyRef,float,float,float>;
            std::map<Key,float> costs;
            RouteOptions oracle;
            oracle.transferCost=[&](IslandId island,Anchor from,Anchor to) {
                if (!from.polygon) from=a.second;
                if (!to.polygon) to=b.second;
                const Key key{island,from.polygon,from.position.x,from.position.y,from.position.z,
                    to.polygon,to.position.x,to.position.y,to.position.z};
                const auto found=costs.find(key);
                if(found!=costs.end()) return found->second;
                const auto cost=referenceCost(graph,mesh,from,to);
                costs.emplace(key,cost);return cost;
            };
            const auto minimum=fixture_oracle::minimumCost(graph,a.first,b.first,a.second.position,b.second.position,oracle);
            require(std::isfinite(minimum)==bool(result.route.value),"Native oracle reachability differs");
            if(result.route.value) {
                require(std::abs(minimum-result.route.value->totalCost)<=1e-5*std::max(1.0,minimum),"Native oracle cost differs");
                double cost=0;Anchor at=a.second;
                for(const auto& leg:result.route.value->legs) {
                    require(leg.crossing<graph.crossings().size(),"Native leg crossing missing");
                    const auto& c=graph.crossings()[leg.crossing];
                    require((leg.reverse?c.traversableBA:c.traversableAB) &&
                        fixture_oracle::same(leg.from,leg.reverse?c.crossing.b:c.crossing.a) &&
                        fixture_oracle::same(leg.to,leg.reverse?c.crossing.a:c.crossing.b),"Native leg invalid");
                    cost+=oracle.transferCost(at.island,at,leg.from)+fixture_oracle::distance(leg.from.position,leg.to.position);
                    at=leg.to;
                }
                require(at.island==b.first,"Native route ends on wrong island");
                cost+=oracle.transferCost(at.island,at,b.second);
                require(std::abs(cost-result.route.value->totalCost)<=1e-5*std::max(1.0,cost),"Native route leg costs differ");
            }
        }
        outcomes.push_back({a.first,b.first,std::move(result),ms});
    }
    return outcomes;
}
}
