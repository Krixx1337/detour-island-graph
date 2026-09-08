#pragma once
#include <detour_island_graph/v2/Routing.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fixture_oracle {
using namespace detour_island_graph::v2;
inline void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
inline double distance(Point a, Point b) {
    return std::sqrt((double(a.x)-b.x)*(double(a.x)-b.x)+(double(a.y)-b.y)*(double(a.y)-b.y)+(double(a.z)-b.z)*(double(a.z)-b.z));
}
inline bool same(const Anchor& a, const Anchor& b) {
    return a.island==b.island && a.polygon==b.polygon && a.position.x==b.position.x && a.position.y==b.position.y && a.position.z==b.position.z;
}
inline void checkRoute(const CompiledGraph& graph, IslandId from, IslandId to, Point start, Point end, const Route& route) {
    require(!route.legs.empty(),"Successful route has no legs");
    double cost=0; Point position=start; auto island=from;
    for (const auto& leg:route.legs) {
        require(leg.crossing<graph.crossings().size(),"Route crossing out of range");
        const auto& c=graph.crossings()[leg.crossing];
        require(leg.reverse?c.traversableBA:c.traversableAB,"Route uses disabled direction");
        require(same(leg.from,leg.reverse?c.crossing.b:c.crossing.a) && same(leg.to,leg.reverse?c.crossing.a:c.crossing.b),"Route anchor mismatch");
        require(leg.from.island==island,"Route island chain broken");
        cost+=distance(position,leg.from.position)+distance(leg.from.position,leg.to.position);
        island=leg.to.island; position=leg.to.position;
    }
    require(island==to,"Route ends on wrong island"); cost+=distance(position,end);
    require(std::isfinite(route.totalCost) && route.totalCost>=0 && std::abs(cost-route.totalCost)<=1e-5*std::max(1.0,cost),"Route cost mismatch");
}
struct PairOutcome {
    IslandId from,to;
    RouteStatus status;
    std::size_t legs,expanded,queued,peakOpen;
    std::optional<float> cost;
};
inline std::vector<PairOutcome> checkAllRoutes(const CompiledGraph& graph, const dtNavMesh& mesh) {
    const auto n=graph.domain().size();
    std::vector<dtPolyRef> refs(n); std::vector<Point> positions(n);
    for(auto p:graph.polygonIslands()) if(p.second<n && (!refs[p.second] || p.first<refs[p.second])) refs[p.second]=p.first;
    for(std::size_t i=0;i<n;++i) if(graph.includes(static_cast<IslandId>(i))) {
        const dtMeshTile* tile=nullptr;const dtPoly* poly=nullptr;
        require(dtStatusSucceed(mesh.getTileAndPolyByRef(refs[i],&tile,&poly)) && poly->vertCount>0,"Invalid route endpoint polygon");
        require(poly->getType()==DT_POLYTYPE_GROUND,"Route endpoint is not ground");
        for(unsigned j=0;j<poly->vertCount;++j) {
            const auto* p=&tile->verts[poly->verts[j]*3];
            positions[i].x+=p[0]/poly->vertCount;positions[i].y+=p[1]/poly->vertCount;positions[i].z+=p[2]/poly->vertCount;
        }
    }
    std::vector<std::vector<IslandId>> edges(n);
    for(const auto& c:graph.crossings()) {
        require(c.crossing.a.island<n && c.crossing.b.island<n,"Oracle endpoint out of range");
        require(mesh.isValidPolyRef(c.crossing.a.polygon) && mesh.isValidPolyRef(c.crossing.b.polygon),"Crossing ref absent from mesh");
        if(c.traversableAB) edges[c.crossing.a.island].push_back(c.crossing.b.island);
        if(c.traversableBA) edges[c.crossing.b.island].push_back(c.crossing.a.island);
    }
    std::vector<PairOutcome> outcomes; RouteScratch scratch;
    for(IslandId from=0;from<n;++from) if(graph.includes(from)) {
        std::vector<bool> reachable(n); std::vector<IslandId> queue{from};reachable[from]=true;
        for(std::size_t j=0;j<queue.size();++j) for(auto next:edges[queue[j]]) if(!reachable[next]) {reachable[next]=true;queue.push_back(next);}
        for(IslandId to=0;to<n;++to) if(graph.includes(to)) {
            RouteOptions options;options.maxExpandedPortals=100000;options.maxQueuedPortals=1000000;
            auto r=findRoute(graph,from,to,positions[from],positions[to],options,&scratch);
            auto expected=from==to?RouteStatus::SameIsland:reachable[to]?RouteStatus::Success:RouteStatus::NoPath;
            require(r.status==expected,"Routing disagrees with independent BFS");
            if(r.status==RouteStatus::Success) {
                require(r.value.has_value(),"Missing successful route");
                require(r.stats.estimatedTransferCost && r.stats.estimatedCrossingCost,"Default cost provenance changed");
                checkRoute(graph,from,to,positions[from],positions[to],*r.value);
            } else require(!r.value,"Unexpected route for non-success status");
            outcomes.push_back({from,to,r.status,r.value?r.value->legs.size():0,r.stats.expandedPortals,
                r.stats.queuedPortals,r.stats.peakOpenSetSize,r.value?std::optional<float>(r.value->totalCost):std::nullopt});
        }
    }
    return outcomes;
}
}
