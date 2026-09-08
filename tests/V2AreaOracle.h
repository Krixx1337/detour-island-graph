#pragma once
#include "V2RouteOracle.h"

namespace area_fixture {
using namespace detour_island_graph::v2;
struct Profile { const char* name; IslandAreaPreference preference; };
inline std::vector<Profile> profiles() {
    return {{"neutral",{}},{"soft",{100,10,0}},{"strict",{0,0,25}},{"combined",{100,10,25}}};
}
// Independent test implementation. Never invokes production area-policy code.
inline double penalty(const CompiledGraph& g, IslandId id, IslandId start, IslandId end,
    IslandAreaPreference p) {
    if (id==start || id==end || p.maxEntryPenalty==0) return 0;
    const double a=g.metrics()[id].surfaceArea/g.identity().unitsPerMeter/g.identity().unitsPerMeter;
    if(a>=p.preferredAreaSquareMeters) return 0;
    return double(p.maxEntryPenalty)*(p.preferredAreaSquareMeters-a)/p.preferredAreaSquareMeters;
}
inline void configure(RouteOptions& o, IslandAreaPreference p) {
    o.crossingFilter=[p](const CompiledCrossing& c,bool reverse,const RouteCostContext& ctx) {
        const auto id=(reverse?c.crossing.a:c.crossing.b).island;
        if(id==ctx.startIsland || id==ctx.endIsland || p.minimumIntermediateAreaSquareMeters==0) return true;
        const double a=ctx.graph.metrics()[id].surfaceArea/ctx.graph.identity().unitsPerMeter/ctx.graph.identity().unitsPerMeter;
        return a>=p.minimumIntermediateAreaSquareMeters;
    };
    o.crossingCost=[p](const CompiledCrossing& c,bool reverse,const RouteCostContext& ctx) {
        return float(fixture_oracle::distance(c.crossing.a.position,c.crossing.b.position))+
            float(penalty(ctx.graph,(reverse?c.crossing.a:c.crossing.b).island,ctx.startIsland,ctx.endIsland,p));
    };
}
inline bool sameLegs(const RouteResult& a,const RouteResult& b) {
    if(a.status!=b.status || bool(a.value)!=bool(b.value)) return false;
    if(!a.value) return true;
    if(a.value->legs.size()!=b.value->legs.size()) return false;
    for(std::size_t i=0;i<a.value->legs.size();++i)
        if(a.value->legs[i].crossing!=b.value->legs[i].crossing || a.value->legs[i].reverse!=b.value->legs[i].reverse) return false;
    return true;
}
}
