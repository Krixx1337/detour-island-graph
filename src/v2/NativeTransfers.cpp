#include <detour_island_graph/v2/NativeTransfers.h>
#include <DetourNavMeshQuery.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>

#ifndef DT_VIRTUAL_QUERYFILTER
#error Native transfers require Detour compiled with DT_VIRTUAL_QUERYFILTER
#endif

namespace detour_island_graph::v2 {
namespace {
struct Abort { TransferStatus status; };
void checkpoint(const Cancel& canceled) {
    if (canceled && canceled()) throw Abort{TransferStatus::Canceled};
}
double distance(const float* a, const float* b) {
    const double x = double(a[0])-b[0], y = double(a[1])-b[1], z = double(a[2])-b[2];
    return std::sqrt(x*x+y*y+z*z);
}
TransferStatus status(dtStatus s) {
    if (dtStatusDetail(s, DT_OUT_OF_MEMORY)) return TransferStatus::OutOfMemory;
    if (dtStatusDetail(s, DT_OUT_OF_NODES) || dtStatusDetail(s, DT_BUFFER_TOO_SMALL))
        return TransferStatus::BudgetExceeded;
    if (dtStatusFailed(s)) return TransferStatus::InvalidInput;
    if (dtStatusDetail(s, DT_PARTIAL_RESULT)) return TransferStatus::Blocked;
    return TransferStatus::Success;
}
RouteStatus routeStatus(TransferStatus s) {
    switch(s) {
    case TransferStatus::Success: return RouteStatus::Success;
    case TransferStatus::Blocked: return RouteStatus::NoPath;
    case TransferStatus::BudgetExceeded: return RouteStatus::BudgetExceeded;
    case TransferStatus::Canceled: return RouteStatus::Canceled;
    case TransferStatus::OutOfMemory: return RouteStatus::OutOfMemory;
    case TransferStatus::CallbackFailed: return RouteStatus::CallbackFailed;
    default: return RouteStatus::InvalidInput;
    }
}
struct IslandFilter final : dtQueryFilter {
    const CompiledGraph* graph = nullptr;
    IslandId island = 0;
    bool passFilter(dtPolyRef ref, const dtMeshTile*, const dtPoly* poly) const override {
        const auto owner = graph->polygonIslands().find(ref);
        return poly->getType() == DT_POLYTYPE_GROUND &&
            owner != graph->polygonIslands().end() && owner->second == island;
    }
};
using Key = std::tuple<IslandId, dtPolyRef, float, float, float, dtPolyRef, float, float, float>;
Key key(IslandId island, const Anchor& a, const Anchor& b) {
    return {island,a.polygon,a.position.x,a.position.y,a.position.z,
        b.polygon,b.position.x,b.position.y,b.position.z};
}
}

struct NativeTransferProvider::Impl {
    const dtNavMesh* mesh = nullptr;
    const CompiledGraph* graph = nullptr;
    NativeTransferOptions options;
    NativeTransferStats stats;
    std::unique_ptr<dtNavMeshQuery, decltype(&dtFreeNavMeshQuery)> query{nullptr,dtFreeNavMeshQuery};
    IslandFilter filter;
    std::vector<dtPolyRef> corridor;
    std::vector<float> points;
    std::map<Key,TransferResult> cache;
    bool ready = false;
    int nodeCapacity = 0;

    void project(const Anchor& a, float* projected) {
        checkpoint(options.canceled);
        if (!ready || !a.polygon || !std::isfinite(a.position.x) ||
            !std::isfinite(a.position.y) || !std::isfinite(a.position.z))
            throw Abort{TransferStatus::InvalidInput};
        const auto owner = graph->polygonIslands().find(a.polygon);
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        if (owner == graph->polygonIslands().end() || owner->second != a.island ||
            !graph->includes(a.island) ||
            dtStatusFailed(mesh->getTileAndPolyByRef(a.polygon,&tile,&poly)) ||
            poly->getType() != DT_POLYTYPE_GROUND)
            throw Abort{TransferStatus::InvalidInput};
        const float p[]{a.position.x,a.position.y,a.position.z};
        bool over = false;
        const auto s = status(query->closestPointOnPoly(a.polygon,p,projected,&over));
        if (s != TransferStatus::Success) throw Abort{s};
        const auto displacement = distance(p,projected);
        if (std::isfinite(displacement)) stats.maxProjectionDistance =
            (std::max)(stats.maxProjectionDistance,displacement);
        if (!std::isfinite(projected[0]) || !std::isfinite(projected[1]) ||
            !std::isfinite(projected[2]) || displacement>options.projectionTolerance)
            throw Abort{TransferStatus::InvalidInput};
    }
};

NativeTransferProvider::NativeTransferProvider() = default;
NativeTransferProvider::~NativeTransferProvider() = default;

TransferStatus NativeTransferProvider::begin(const dtNavMesh& mesh, const CompiledGraph& graph,
    std::uint64_t identity, const NativeTransferOptions& options) {
    try {
        if (!impl_) impl_ = std::make_unique<Impl>();
        auto& w = *impl_;
        w.ready = false;
        w.cache.clear();
        w.stats = {};
        w.options = options;
        checkpoint(options.canceled);
        if (identity != graph.identity().mesh || !options.maxQueries || !options.maxIterations ||
            options.maxNodes < 4 || options.maxNodes > 65535 || options.maxCorridor <= 0 ||
            options.maxStraightPoints <= 0 ||
            options.maxStraightPoints > (std::numeric_limits<int>::max)()/3 ||
            !std::isfinite(options.projectionTolerance) ||
            options.projectionTolerance < 0)
            return TransferStatus::InvalidInput;
        // Detour init retains larger node pools. Recreate when the requested
        // cap changes so scratch reuse cannot silently weaken a smaller limit.
        if (w.nodeCapacity != options.maxNodes) {
            w.query.reset();
            w.nodeCapacity = options.maxNodes;
        }
        if (!w.query) w.query.reset(dtAllocNavMeshQuery());
        if (!w.query) return TransferStatus::OutOfMemory;
        const auto initialized = status(w.query->init(&mesh,options.maxNodes));
        if (initialized != TransferStatus::Success) return initialized;
        w.corridor.resize(std::size_t(options.maxCorridor));
        w.points.resize(std::size_t(options.maxStraightPoints)*3);
        w.mesh = &mesh;
        w.graph = &graph;
        w.filter.graph = &graph;
        checkpoint(options.canceled);
        w.ready = true;
        return TransferStatus::Success;
    } catch (const Abort& e) { return e.status; }
    catch (const std::bad_alloc&) { return TransferStatus::OutOfMemory; }
    catch (...) { return TransferStatus::CallbackFailed; }
}

TransferStatus NativeTransferProvider::checkAnchor(const Anchor& anchor) {
    try {
        if (!impl_ || !impl_->ready) return TransferStatus::InvalidInput;
        float p[3]; impl_->project(anchor,p);
        checkpoint(impl_->options.canceled);
        return TransferStatus::Success;
    } catch (const Abort& e) { return e.status; }
    catch (const std::bad_alloc&) { return TransferStatus::OutOfMemory; }
    catch (...) { return TransferStatus::CallbackFailed; }
}

TransferResult NativeTransferProvider::evaluate(IslandId island, const Anchor& from, const Anchor& to) {
    try {
        if (!impl_ || !impl_->ready || from.island != island || to.island != island)
            return {TransferStatus::InvalidInput};
        auto& w = *impl_;
        float a[3], b[3];
        w.project(from,a); w.project(to,b);
        const auto k = key(island,from,to);
        const auto cached = w.cache.find(k);
        if (cached != w.cache.end()) {
            ++w.stats.cacheHits;
            checkpoint(w.options.canceled);
            return cached->second;
        }
        ++w.stats.cacheMisses;
        if (w.stats.queries >= w.options.maxQueries) return {TransferStatus::BudgetExceeded};
        ++w.stats.queries;
        w.filter.island = island;
        auto s = w.query->initSlicedFindPath(from.polygon,to.polygon,a,b,&w.filter);
        while (dtStatusInProgress(s)) {
            checkpoint(w.options.canceled);
            if (w.stats.iterations >= w.options.maxIterations)
                return {TransferStatus::BudgetExceeded};
            int iterations = 0;
            const auto batch = int((std::min)(std::size_t(32),w.options.maxIterations-w.stats.iterations));
            s = w.query->updateSlicedFindPath(batch,&iterations);
            w.stats.iterations += std::size_t(iterations);
        }
        auto outcome = status(s);
        int count = 0;
        if (outcome == TransferStatus::Success) {
            s = w.query->finalizeSlicedFindPath(w.corridor.data(),&count,w.options.maxCorridor);
            outcome = status(s);
            w.stats.peakCorridor = (std::max)(w.stats.peakCorridor,std::size_t(count));
        }
        if (outcome != TransferStatus::Success && outcome != TransferStatus::Blocked)
            return {outcome};
        TransferResult result{outcome};
        if (outcome == TransferStatus::Success) {
            if (!count || w.corridor[0] != from.polygon || w.corridor[count-1] != to.polygon)
                return {TransferStatus::InvalidInput};
            for (int i=0; i<count; ++i) {
                checkpoint(w.options.canceled);
                const dtMeshTile* tile = nullptr;
                const dtPoly* poly = nullptr;
                if (dtStatusFailed(w.mesh->getTileAndPolyByRef(w.corridor[i],&tile,&poly)) ||
                    !w.filter.passFilter(w.corridor[i],tile,poly)) return {TransferStatus::InvalidInput};
            }
            int points = 0;
            s = w.query->findStraightPath(a,b,w.corridor.data(),count,w.points.data(),nullptr,nullptr,
                &points,w.options.maxStraightPoints,DT_STRAIGHTPATH_ALL_CROSSINGS);
            w.stats.peakStraightPoints = (std::max)(w.stats.peakStraightPoints,std::size_t(points));
            outcome = status(s);
            if (outcome != TransferStatus::Success) return {outcome};
            if (points <= 0) return {TransferStatus::InvalidInput};
            double cost = 0;
            for (int i=1; i<points; ++i) {
                checkpoint(w.options.canceled);
                cost += distance(&w.points[std::size_t(i-1)*3],&w.points[std::size_t(i)*3]);
            }
            result.cost = float(cost);
            if (!std::isfinite(result.cost)) return {TransferStatus::InvalidInput};
        }
        checkpoint(w.options.canceled);
        if (result.status == TransferStatus::Blocked) ++w.stats.blocked;
        if (w.cache.size()<w.options.cacheEntries) {
            w.cache.emplace(k,result);
            w.stats.cacheEntries = w.cache.size();
        }
        return result;
    } catch (const Abort& e) { return {e.status}; }
    catch (const std::bad_alloc&) { return {TransferStatus::OutOfMemory}; }
    catch (...) { return {TransferStatus::CallbackFailed}; }
}

NativeTransferStats NativeTransferProvider::stats() const { return impl_ ? impl_->stats : NativeTransferStats{}; }

NativeRouteResult findNativeRoute(const CompiledGraph& graph, const dtNavMesh& mesh,
    std::uint64_t identity, Anchor start, Anchor end, const NativeTransferOptions& nativeOptions,
    const RouteOptions& routeOptions, NativeTransferProvider* provider, RouteScratch* scratch) {
    NativeRouteResult result;
    NativeTransferProvider local;
    auto& work = provider ? *provider : local;
    try {
        auto options = nativeOptions;
        options.canceled = [nativeCancel = nativeOptions.canceled, routeCancel = routeOptions.canceled] {
            return (nativeCancel && nativeCancel()) || (routeCancel && routeCancel());
        };
        const auto initialized = work.begin(mesh,graph,identity,options);
        if (initialized != TransferStatus::Success) result.route.status = routeStatus(initialized);
        else if (routeOptions.transferCost || routeOptions.transferEvaluator)
            result.route.status = RouteStatus::InvalidInput;
        else {
            auto checked = work.checkAnchor(start);
            if (checked == TransferStatus::Success) checked = work.checkAnchor(end);
            if (checked != TransferStatus::Success) result.route.status = routeStatus(checked);
            else {
                auto routing = routeOptions;
                routing.transferEvaluator = [&](IslandId i, const Anchor& a, const Anchor& b) {
                    return work.evaluate(i,a,b);
                };
                routing.transferCostEstimated = false;
                routing.canceled = options.canceled;
                result.route = findRoute(graph,start,end,routing,scratch);
            }
        }
    } catch (const std::bad_alloc&) { result.route.status = RouteStatus::OutOfMemory; }
    catch (...) { result.route.status = RouteStatus::CallbackFailed; }
    result.transfers = work.stats();
    return result;
}
} // namespace detour_island_graph::v2
