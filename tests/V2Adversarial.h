#pragma once
#include "V2RouteOracle.h"
#include <detour_island_graph/v2/Health.h>
#include <detour_island_graph/v2/Serialization.h>
#include <DetourNavMeshBuilder.h>
#include <set>
#include <tuple>
#include <sstream>
#include <chrono>

// Procedural, test-only worlds. Coordinates use an exact quarter-unit grid.
namespace adversarial {
using namespace detour_island_graph::v2;
using fixture_oracle::require;
struct Rect { float x0,z0,x1,z1,y=0; };
struct DeleteMesh { void operator()(dtNavMesh* p) const { dtFreeNavMesh(p); } };
using Mesh=std::unique_ptr<dtNavMesh,DeleteMesh>;
inline Mesh makeMesh(const std::vector<Rect>& rectangles) {
    require(!rectangles.empty() && rectangles.size()<16000,"Invalid procedural rectangle count");
    std::vector<unsigned short> vertices,polygons,flags(rectangles.size(),1);
    std::vector<unsigned char> areas(rectangles.size(),0);
    Point maximum{};
    for(const auto& r:rectangles) {
        require(r.x1>r.x0 && r.z1>r.z0,"Invalid rectangle bounds");
        const auto base=static_cast<unsigned short>(vertices.size()/3);
        for(Point p:std::vector<Point>{{r.x0,r.y,r.z0},{r.x1,r.y,r.z0},{r.x1,r.y,r.z1},{r.x0,r.y,r.z1}}) {
            for(float value:{p.x,p.y,p.z}) {
                require(std::isfinite(value) && value>=0 && value*4<65535 && std::floor(value*4)==value*4,"Rectangle must use quarter grid");
                vertices.push_back(static_cast<unsigned short>(value*4));
            }
            maximum.x=std::max(maximum.x,p.x);maximum.y=std::max(maximum.y,p.y);maximum.z=std::max(maximum.z,p.z);
        }
        for(unsigned short j=0;j<4;++j)polygons.push_back(base+j);
        polygons.insert(polygons.end(),4,0xffff);
    }
    dtNavMeshCreateParams p{};p.verts=vertices.data();p.vertCount=int(vertices.size()/3);
    p.polys=polygons.data();p.polyCount=int(rectangles.size());p.nvp=4;p.polyFlags=flags.data();p.polyAreas=areas.data();
    p.cs=p.ch=.25f;p.bmax[0]=maximum.x+1;p.bmax[1]=maximum.y+1;p.bmax[2]=maximum.z+1;
    p.walkableHeight=1.8f;p.walkableRadius=.3f;p.walkableClimb=.5f;p.buildBvTree=true;
    unsigned char* data=nullptr;int size=0;
    require(dtCreateNavMeshData(&p,&data,&size),"Procedural mesh creation failed");
    Mesh mesh(dtAllocNavMesh());
    if(!mesh || dtStatusFailed(mesh->init(data,size,DT_TILE_FREE_DATA))) {dtFree(data);throw std::runtime_error("Procedural mesh init failed");}
    return mesh;
}
inline Anchor anchor(const dtNavMesh& mesh,IslandId i,Point p) {
    return {i,mesh.getPolyRefBase(mesh.getTile(0))|dtPolyRef(i),p};
}
using Endpoint=std::tuple<IslandId,dtPolyRef,float,float,float>;
using Key=std::pair<Endpoint,Endpoint>;
inline Endpoint endpoint(const Anchor& a) {return {a.island,a.polygon,a.position.x,a.position.y,a.position.z};}
inline Key key(const Anchor& a,const Anchor& b) {auto x=endpoint(a),y=endpoint(b);return x<y?Key{x,y}:Key{y,x};}
// Independent rectangle arithmetic: no production sampling or Detour projection.
inline std::set<Key> expected(const std::vector<Rect>& rectangles,const dtNavMesh& mesh,const DiscoveryConfig& config) {
    std::set<Key> result;
    for(IslandId i=0;i<rectangles.size();++i) {
        const auto& r=rectangles[i];std::set<std::pair<float,float>> perimeter;
        const auto nx=static_cast<unsigned>(std::ceil((r.x1-r.x0)/config.sampleSpacing));
        const auto nz=static_cast<unsigned>(std::ceil((r.z1-r.z0)/config.sampleSpacing));
        for(unsigned k=0;k<=nx;++k) {
            float x=float(double(r.x0)+double(r.x1-r.x0)*k/nx);
            perimeter.emplace(x,r.z0);perimeter.emplace(x,r.z1);
        }
        for(unsigned k=0;k<=nz;++k) {
            float z=float(double(r.z0)+double(r.z1-r.z0)*k/nz);
            perimeter.emplace(r.x0,z);perimeter.emplace(r.x1,z);
        }
        for(auto p:perimeter) for(IslandId j=0;j<rectangles.size();++j) if(i!=j) {
            const auto& target=rectangles[j];
            Point a{p.first,r.y,p.second},b{std::clamp(a.x,target.x0,target.x1),target.y,std::clamp(a.z,target.z0,target.z1)};
            const double dy=double(b.y)-a.y;
            if(std::hypot(double(b.x)-a.x,double(b.z)-a.z)>config.maxHorizontalGap)continue;
            if(!((dy<=config.maxClimb && -dy<=config.maxDrop)||(-dy<=config.maxClimb && dy<=config.maxDrop)))continue;
            result.insert(key(anchor(mesh,i,a),anchor(mesh,j,b)));
        }
    }
    return result;
}
inline ProductionBuildOptions production(std::size_t batch=64) {
    // Fixed ceilings for <=1,024 sparse islands and <=256 fully overlapping layers.
    return {{128*1024*1024,10000000,2000,10000,512,500000,500000,1000000},batch,batch};
}
inline std::string bytes(const CompiledGraph& g) {
    std::ostringstream out(std::ios::binary);require(GraphSerializer::write(out,g)==SerializationStatus::Success,"Scenario serialization failed");return out.str();
}
struct Result {
    std::string name;
    std::vector<Rect> geometry;
    DiscoveryConfig config;
    StageStatus status=StageStatus::InvalidInput;
    BudgetResource exhausted=BudgetResource::None;
    std::size_t expectedCandidates=0,missing=0,extra=0,islands=0,directions=0,work=0,bytes=0,nearby=0;
    std::size_t routeExpanded=0,routeQueued=0,limit=0,attempted=0;
    double durationMs=0;
    std::vector<fixture_oracle::PairOutcome> routes;
};
inline void checkGraph(const CompiledGraph& g,const dtNavMesh& mesh,bool small,Result* stats=nullptr) {
    const auto h=analyzeGraphHealth(g);require(h.status==StageStatus::Success && h.value && h.value->healthy(),"Scenario graph unhealthy");
    if(small) {
        for(const auto& pair:fixture_oracle::checkAllRoutes(g,mesh,g.traversals().size()<=512))if(stats){stats->routeExpanded+=pair.expanded;stats->routeQueued+=pair.queued;stats->routes.push_back(pair);}
        return;
    }
    const auto n=static_cast<IslandId>(g.domain().size());
    std::vector<std::vector<IslandId>> edges(n);
    for(const auto& c:g.crossings()) {
        if(c.traversableAB)edges[c.crossing.a.island].push_back(c.crossing.b.island);
        if(c.traversableBA)edges[c.crossing.b.island].push_back(c.crossing.a.island);
    }
    auto center=[&](IslandId island) {
        const auto ref=mesh.getPolyRefBase(mesh.getTile(0))|dtPolyRef(island);
        const dtMeshTile* tile=nullptr;const dtPoly* poly=nullptr;
        require(dtStatusSucceed(mesh.getTileAndPolyByRef(ref,&tile,&poly)),"Stress endpoint missing");
        Point p{};for(unsigned j=0;j<poly->vertCount;++j){const auto* v=&tile->verts[poly->verts[j]*3];p.x+=v[0]/poly->vertCount;p.y+=v[1]/poly->vertCount;p.z+=v[2]/poly->vertCount;}return p;
    };
    RouteScratch scratch,referenceScratch;
    for(auto pair:std::vector<std::pair<IslandId,IslandId>>{{0,n-1},{n-1,0},{0,n/2},{n/2,n-1}}) {
        std::vector<bool> seen(n);std::vector<IslandId> queue{pair.first};seen[pair.first]=true;
        for(std::size_t i=0;i<queue.size();++i)for(auto next:edges[queue[i]])if(!seen[next]){seen[next]=true;queue.push_back(next);}
        RouteOptions o;o.maxExpandedPortals=1000000;o.maxQueuedPortals=2000000;
        const auto start=center(pair.first),end=center(pair.second);
        const auto begin=std::chrono::steady_clock::now();
        auto route=findRoute(g,pair.first,pair.second,start,end,o,&scratch);
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        o.enableArrivalDominance=false;
        const auto referenceBegin=std::chrono::steady_clock::now();
        const auto reference=findRoute(g,pair.first,pair.second,start,end,o,&referenceScratch);
        const double referenceMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-referenceBegin).count();
        fixture_oracle::equivalent(route,reference);
        if(reference.value)fixture_oracle::checkRoute(g,pair.first,pair.second,start,end,*reference.value);
        if(n==256 && g.traversals().size()==261120) {
            require(route.stats.examinedTraversals<=reference.stats.examinedTraversals/10,"Dense routing did not reduce scans tenfold");
            require(route.stats.examinedTraversals<=13213182,"Dense routing exceeded fixed scan ceiling");
        }
        if(stats)stats->routes.push_back({pair.first,pair.second,route.status,route.value?route.value->legs.size():0,
            route.stats.expandedPortals,route.stats.queuedPortals,route.stats.peakOpenSetSize,
            route.value?std::optional<float>(route.value->totalCost):std::nullopt,route.stats,ms,reference.stats,referenceMs});
        if(stats){stats->routeExpanded+=route.stats.expandedPortals;stats->routeQueued+=route.stats.queuedPortals;}
        require(route.status==(seen[pair.second]?RouteStatus::Success:RouteStatus::NoPath),"Stress route differs from BFS");
        if(route.value)fixture_oracle::checkRoute(g,pair.first,pair.second,start,end,*route.value);
    }
}
inline Result run(const std::string& name,const std::vector<Rect>& rects,DiscoveryConfig config,bool compareBatches=true) {
    const auto begin=std::chrono::steady_clock::now();
    auto mesh=makeMesh(rects);BuildInput input;input.navMesh=mesh.get();
    Result result;result.name=name;result.geometry=rects;result.config=config;
    const auto oracle=expected(rects,*mesh,config);result.expectedCandidates=oracle.size();
    std::string first;
    for(auto batch:compareBatches?std::vector<std::size_t>{1,64}:std::vector<std::size_t>{64}) {
        auto built=buildGraphBounded(input,config,production(batch));
        require(built.status==StageStatus::Success && built.graph,"Scenario build failed");
        std::set<Key> actual;
        for(const auto& c:built.graph->crossings())actual.insert(key(c.crossing.a,c.crossing.b));
        for(const auto& k:oracle)result.missing+=actual.count(k)==0;
        for(const auto& k:actual)result.extra+=oracle.count(k)==0;
        require(!result.missing && !result.extra,(name+": analytic discovery oracle mismatch").c_str());
        const auto routeBegin=result.routes.size();
        checkGraph(*built.graph,*mesh,rects.size()<=16,&result);
        const auto routeCount=result.routes.size()-routeBegin;
        const auto serialized=bytes(*built.graph);
        if(!first.empty())require(first==serialized,"Scenario batch mismatch");first=serialized;
        std::istringstream stream(serialized,std::ios::binary);auto decoded=GraphSerializer::read(stream);
        require(decoded.status==SerializationStatus::Success && decoded.graph,"Scenario decode failed");
        require(bytes(*decoded.graph)==serialized,"Scenario round-trip mismatch");checkGraph(*decoded.graph,*mesh,rects.size()<=16,&result);
        require(result.routes.size()==routeBegin+2*routeCount,"Scenario route count changed");
        for(std::size_t i=0;i<routeCount;++i) {
            const auto& original=result.routes[routeBegin+i];const auto& decodedRoute=result.routes[routeBegin+routeCount+i];
            require(original.status==decodedRoute.status && original.cost==decodedRoute.cost &&
                fixture_oracle::sameWork(original.stats,decodedRoute.stats) &&
                fixture_oracle::sameWork(original.referenceStats,decodedRoute.referenceStats),"Scenario decoded route work changed");
            if(routeBegin)require(fixture_oracle::sameWork(result.routes[i].stats,original.stats) &&
                fixture_oracle::sameWork(result.routes[i].referenceStats,original.referenceStats),"Scenario batch route work changed");
        }
        result.status=built.status;result.islands=rects.size();result.directions=built.graph->traversals().size();
        result.work=built.budget.workUnits;result.bytes=built.budget.peakAllocationBytes;result.nearby=built.budget.peakNearbyRefs;
    }
    result.durationMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    return result;
}
inline ProductionBuildResult landingWindow(float spacing) {
    auto mesh=makeMesh({{0,0,2,4},{3,0,5,4}});BuildInput input;input.navMesh=mesh.get();
    ValidationOptions validation;
    // A known landing corridor at z=1 on both facing rectangle edges.
    validation.validator=[](const ValidationRequest& r) {
        return ValidationResult{r.from.position.z==1 && r.to.position.z==1?ValidationState::Valid:ValidationState::Invalid,1};
    };
    auto result=buildGraphBounded(input,{spacing,1,0,0,1000,10000},production(),validation,{CompilePolicy::ValidatedOnly,{}});
    require(result.status==StageStatus::Success && result.graph,"Sampling window build failed");
    require(result.graph->traversals().empty()==(spacing==4),"Sampling window coverage mismatch");
    checkGraph(*result.graph,*mesh,true);
    return result;
}
inline std::vector<Result> runScenarios() {
    std::vector<Result> results;
    auto config=DiscoveryConfig{.5f,1,1,3,20000,1000000};
    results.push_back(run("horizontal_exact",{{0,0,2,2},{3,0,5,2}},config));
    results.push_back(run("horizontal_outside",{{0,0,2,2},{3.25f,0,5.25f,2}},config));
    results.push_back(run("asymmetric_drop",{{0,0,2,2},{3,0,5,2,3}},config));
    results.push_back(run("vertical_outside",{{0,0,2,2},{0,0,2,2,3.25f}},config));
    results.push_back(run("stacked_layers",{{0,0,2,2},{0,0,2,2,150},{0,0,2,2,450}},config));
    results.push_back(run("narrow_platform",{{0,0,2,2},{3,.75f,3.5f,1.25f}},config));
    for(unsigned count:{64u,256u,1024u}) {
        std::vector<Rect> rects;for(unsigned i=0;i<count;++i){float x=float(i%32)*4,z=float(i/32)*4;rects.push_back({x,z,x+1,z+1});}
        results.push_back(run("sparse_"+std::to_string(count),rects,{1,1,1,1,20000,1000000},false));
    }
    for(unsigned count:{16u,64u,256u}) {
        std::vector<Rect> rects;for(unsigned i=0;i<count;++i)rects.push_back({0,0,2,2,float(i)});
        auto dense=DiscoveryConfig{2,.5f,256,256,20000,1000000};
        auto mesh=makeMesh(rects);BuildInput input;input.navMesh=mesh.get();
        auto limits=production();limits.limits.maxNearbyRefsPerQuery=count-1;
        auto failed=buildGraphBounded(input,dense,limits);
        require(failed.status==StageStatus::BudgetExceeded && !failed.graph && failed.budget.exhausted==BudgetResource::NearbyRefsPerQuery,"Dense limit must reject without partial graph");
        Result failure;failure.name="dense_limit_"+std::to_string(count);failure.geometry=rects;failure.config=dense;
        failure.status=failed.status;failure.exhausted=failed.budget.exhausted;failure.nearby=failed.budget.peakNearbyRefs;
        failure.islands=count;failure.limit=failed.budget.limit;failure.attempted=failed.budget.attempted;
        failure.work=failed.budget.workUnits;failure.bytes=failed.budget.peakAllocationBytes;results.push_back(failure);
        results.push_back(run("dense_"+std::to_string(count),rects,dense,count<=16));
    }
    return results;
}
}
