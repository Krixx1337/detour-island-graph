#include <detour_island_graph/v2/Build.h>

#include <DetourNavMeshQuery.h>
#include <DetourStatus.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <vector>

namespace detour_island_graph::v2 {
namespace {

struct Abort { StageStatus status; };

void require(bool condition) {
    if (!condition) throw Abort{StageStatus::InvalidInput};
}

void checkpoint(const Cancel& canceled) {
    if (canceled && canceled()) throw Abort{StageStatus::Canceled};
}

bool finite(Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

void normalizeZero(float& value) {
    if (value == 0) value = 0;
}

void validateConfig(const DiscoveryConfig& config) {
    require(std::isfinite(config.sampleSpacing) && config.sampleSpacing > 0);
    for (float limit : {config.maxHorizontalGap, config.maxClimb, config.maxDrop})
        require(std::isfinite(limit) && limit >= 0);
}

std::unordered_map<dtPolyRef, IslandId> indexTopology(
    const TopologyArtifact& topology, const Cancel& cancel) {
    require(std::isfinite(topology.identity.unitsPerMeter) && topology.identity.unitsPerMeter > 0);
    require(topology.islandCount <= topology.polygons.size());
    require(topology.islandCount <= (std::numeric_limits<IslandId>::max)());
    std::unordered_map<dtPolyRef, IslandId> index;
    std::vector<bool> seen(topology.islandCount, false);
    for (const auto& entry : topology.polygons) {
        checkpoint(cancel);
        require(entry.polygon != 0 && entry.island < topology.islandCount);
        require(index.emplace(entry.polygon, entry.island).second);
        seen[entry.island] = true;
    }
    for (bool present : seen) {
        checkpoint(cancel);
        require(present);
    }
    return index;
}

bool withinLimits(Point from, Point to, const DiscoveryConfig& config) {
    const double dx = double(to.x) - from.x;
    const double dz = double(to.z) - from.z;
    const double dy = double(to.y) - from.y;
    return std::hypot(dx, dz) <= config.maxHorizontalGap &&
        dy <= config.maxClimb && -dy <= config.maxDrop;
}

// Discovery keeps a pair when at least one traversal direction is geometrically
// eligible. Checking only sample->landing would discard reverse-valid asymmetric
// pairs that the reverse sample need not reproduce with identical geometry.
bool eligibleEitherWay(Point a, Point b, const DiscoveryConfig& config) {
    return withinLimits(a, b, config) || withinLimits(b, a, config);
}

class Collector final : public dtPolyQuery {
public:
    explicit Collector(std::vector<dtPolyRef>& output) : output_(output) {}

    void process(const dtMeshTile*, dtPoly**, dtPolyRef* refs, int count) override {
        output_.insert(output_.end(), refs, refs + count);
    }

private:
    std::vector<dtPolyRef>& output_;
};

struct QueryDeleter {
    void operator()(dtNavMeshQuery* query) const {
        if (query) dtFreeNavMeshQuery(query);
    }
};

} // namespace

StageResult<std::vector<CrossingCandidate>> discoverCandidates(
    const SamplingArtifact& sampling, const dtNavMesh& navMesh,
    const DiscoveryConfig& config, const Cancel& canceled) {
    StageResult<std::vector<CrossingCandidate>> result;
    try {
        checkpoint(canceled);
        validateConfig(config);
        const auto index = indexTopology(sampling.topology, canceled);
        for (const auto& sample : sampling.samples) {
            checkpoint(canceled);
            require(finite(sample.position));
            const auto it = index.find(sample.polygon);
            require(it != index.end() && it->second == sample.island);
            require(sample.island < sampling.topology.islandCount);
        }

        dtNavMeshQuery* raw = dtAllocNavMeshQuery();
        if (!raw) throw Abort{StageStatus::OutOfMemory};
        std::unique_ptr<dtNavMeshQuery, QueryDeleter> query(raw);
        const dtStatus initStatus = query->init(&navMesh, 256);
        if (dtStatusFailed(initStatus)) {
            if (initStatus & DT_OUT_OF_MEMORY) throw Abort{StageStatus::OutOfMemory};
            throw Abort{StageStatus::InvalidInput};
        }

        dtQueryFilter filter;
        const float verticalExtent = (std::max)(config.maxClimb, config.maxDrop);
        std::vector<CrossingCandidate> candidates;
        std::vector<dtPolyRef> nearby;
        nearby.reserve(64);

        for (const auto& sample : sampling.samples) {
            checkpoint(canceled);
            const float center[3] = {sample.position.x, sample.position.y, sample.position.z};
            const float extents[3] = {config.maxHorizontalGap, verticalExtent, config.maxHorizontalGap};
            nearby.clear();
            Collector collector(nearby);
            ++result.stats.discoveryQueries;
            const dtStatus queryStatus = query->queryPolygons(center, extents, &filter, &collector);
            if (dtStatusFailed(queryStatus)) throw Abort{StageStatus::InvalidInput};
            checkpoint(canceled);
            result.stats.nearbyPolygons += nearby.size();
            std::sort(nearby.begin(), nearby.end());
            nearby.erase(std::unique(nearby.begin(), nearby.end()), nearby.end());
            for (dtPolyRef ref : nearby) {
                checkpoint(canceled);
                if (ref == 0 || ref == sample.polygon) continue;
                const auto found = index.find(ref);
                if (found == index.end()) continue;
                if (found->second == sample.island) continue;
                float closest[3] = {};
                bool overPoly = false;
                if (dtStatusFailed(query->closestPointOnPoly(ref, center, closest, &overPoly))) {
                    ++result.stats.projectionFailures;
                    continue;
                }
                (void)overPoly;
                if (!std::isfinite(closest[0]) || !std::isfinite(closest[1]) || !std::isfinite(closest[2]))
                    throw Abort{StageStatus::InvalidInput};
                normalizeZero(closest[0]);
                normalizeZero(closest[1]);
                normalizeZero(closest[2]);
                ++result.stats.projections;
                const Point landing{closest[0], closest[1], closest[2]};
                if (!eligibleEitherWay(sample.position, landing, config)) continue;
                CrossingCandidate candidate{sample, {found->second, ref, landing}};
                candidates.push_back(candidate);
                ++result.stats.candidatesVisited;
                if (config.maxCandidates && candidates.size() > config.maxCandidates)
                    throw Abort{StageStatus::BudgetExceeded};
            }
        }
        checkpoint(canceled);
        result.value.emplace(std::move(candidates));
        result.status = StageStatus::Success;
    } catch (const Abort& error) { result.status = error.status; }
    catch (const std::bad_alloc&) { result.status = StageStatus::OutOfMemory; }
    catch (...) { result.status = StageStatus::CallbackFailed; }
    return result;
}

} // namespace detour_island_graph::v2
