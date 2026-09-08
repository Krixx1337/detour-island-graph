#pragma once
#include <detour_island_graph/v2/Routing.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <chrono>

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
    RouteStats stats;
    double routeMs=0;
    RouteStats referenceStats;
    double referenceMs=0;
};
inline bool sameWork(const RouteStats& a,const RouteStats& b) {
    return a.expandedPortals==b.expandedPortals && a.queuedPortals==b.queuedPortals &&
        a.peakOpenSetSize==b.peakOpenSetSize && a.examinedTraversals==b.examinedTraversals &&
        a.transferEvaluations==b.transferEvaluations && a.crossingEvaluations==b.crossingEvaluations &&
        a.heapPops==b.heapPops && a.staleHeapPops==b.staleHeapPops &&
        a.usedArrivalDominance==b.usedArrivalDominance && a.arrivalGroups==b.arrivalGroups &&
        a.dominatedArrivals==b.dominatedArrivals && a.arrivalScratchBytes==b.arrivalScratchBytes;
}
inline void equivalent(const RouteResult& optimized,const RouteResult& reference) {
    require(optimized.status==reference.status,"Optimized routing status differs from reference");
    require(bool(optimized.value)==bool(reference.value),"Optimized route presence differs");
    if(optimized.value)require(std::abs(double(optimized.value->totalCost)-reference.value->totalCost)<=
        1e-5*std::max(1.0,double(reference.value->totalCost)),"Optimized route cost differs from reference");
}
// Independent O(P^2) Dijkstra. Directed states come from crossings, never
// production adjacency or search scratch. Restricted to small fixture graphs.
inline double minimumCost(const CompiledGraph& graph, IslandId from, IslandId to,
    Point start, Point end, const RouteOptions& options = {}) {
    struct Edge { Anchor a,b; std::size_t crossing; bool reverse; };
    std::vector<Edge> edges;
    RouteCostContext context{graph,from,to};
    for(std::size_t i=0;i<graph.crossings().size();++i) {
        const auto& c=graph.crossings()[i];
        for(bool reverse:{false,true}) if(reverse?c.traversableBA:c.traversableAB) {
            if(options.crossingFilter && !options.crossingFilter(c,reverse,context))continue;
            edges.push_back({reverse?c.crossing.b:c.crossing.a,reverse?c.crossing.a:c.crossing.b,i,reverse});
        }
    }
    const double infinity=std::numeric_limits<double>::infinity();
    auto valid=[](double c){return std::isfinite(c)&&c>=0;};
    auto transfer=[&](const Anchor& a,const Anchor& b)->double {
        return options.transferCost?options.transferCost(a.island,a,b):distance(a.position,b.position);
    };
    auto gap=[&](const Edge& e)->double {
        return options.crossingCost?options.crossingCost(graph.crossings()[e.crossing],e.reverse,context):distance(e.a.position,e.b.position);
    };
    std::vector<double> costs(edges.size(),infinity);
    std::vector<bool> done(edges.size());
    for(std::size_t i=0;i<edges.size();++i) if(edges[i].a.island==from) {
        const auto t=transfer({from,0,start},edges[i].a),g=gap(edges[i]);
        if(valid(t)&&valid(g))costs[i]=t+g;
    }
    double best=infinity;
    for(std::size_t step=0;step<edges.size();++step) {
        std::size_t next=edges.size();
        for(std::size_t i=0;i<edges.size();++i)if(!done[i]&&(next==edges.size()||costs[i]<costs[next]))next=i;
        if(next==edges.size()||!std::isfinite(costs[next]))break;
        done[next]=true;const auto& at=edges[next].b;
        if(at.island==to) {auto t=transfer(at,{to,0,end});if(valid(t))best=std::min(best,costs[next]+t);}
        for(std::size_t i=0;i<edges.size();++i)if(!done[i]&&edges[i].a.island==at.island) {
            const auto t=transfer(at,edges[i].a),g=gap(edges[i]);
            if(valid(t)&&valid(g))costs[i]=std::min(costs[i],costs[next]+t+g);
        }
    }
    return best;
}
inline std::vector<PairOutcome> checkAllRoutes(const CompiledGraph& graph, const dtNavMesh& mesh, bool optimality=true) {
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
    std::vector<PairOutcome> outcomes; RouteScratch scratch,referenceScratch;
    for(IslandId from=0;from<n;++from) if(graph.includes(from)) {
        std::vector<bool> reachable(n); std::vector<IslandId> queue{from};reachable[from]=true;
        for(std::size_t j=0;j<queue.size();++j) for(auto next:edges[queue[j]]) if(!reachable[next]) {reachable[next]=true;queue.push_back(next);}
        for(IslandId to=0;to<n;++to) if(graph.includes(to)) {
            RouteOptions options;options.maxExpandedPortals=100000;options.maxQueuedPortals=1000000;
            const auto begin=std::chrono::steady_clock::now();
            auto r=findRoute(graph,from,to,positions[from],positions[to],options,&scratch);
            const double routeMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
            options.enableArrivalDominance=false;
            const auto referenceBegin=std::chrono::steady_clock::now();
            const auto reference=findRoute(graph,from,to,positions[from],positions[to],options,&referenceScratch);
            const double referenceMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-referenceBegin).count();
            equivalent(r,reference);
            if(reference.value)checkRoute(graph,from,to,positions[from],positions[to],*reference.value);
            auto expected=from==to?RouteStatus::SameIsland:reachable[to]?RouteStatus::Success:RouteStatus::NoPath;
            require(r.status==expected,"Routing disagrees with independent BFS");
            if(r.status==RouteStatus::Success) {
                require(r.value.has_value(),"Missing successful route");
                require(r.stats.estimatedTransferCost && r.stats.estimatedCrossingCost,"Default cost provenance changed");
                checkRoute(graph,from,to,positions[from],positions[to],*r.value);
                if(optimality) {
                    const auto minimum=minimumCost(graph,from,to,positions[from],positions[to]);
                    require(std::isfinite(minimum)&&std::abs(minimum-r.value->totalCost)<=1e-5*std::max(1.0,minimum),"Route is not minimum cost");
                }
            } else require(!r.value,"Unexpected route for non-success status");
            outcomes.push_back({from,to,r.status,r.value?r.value->legs.size():0,r.stats.expandedPortals,
                r.stats.queuedPortals,r.stats.peakOpenSetSize,r.value?std::optional<float>(r.value->totalCost):std::nullopt,r.stats,routeMs,reference.stats,referenceMs});
        }
    }
    return outcomes;
}
}
