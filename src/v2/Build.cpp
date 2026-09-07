#include <detour_island_graph/v2/Build.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <new>
#include <tuple>
#include <utility>

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

void normalizeZero(Point& p) {
    if (p.x == 0) p.x = 0;
    if (p.y == 0) p.y = 0;
    if (p.z == 0) p.z = 0;
}

bool validState(ValidationState state) {
    return state == ValidationState::Unknown || state == ValidationState::Valid ||
        state == ValidationState::Invalid;
}

void validateConfig(const DiscoveryConfig& config) {
    require(std::isfinite(config.sampleSpacing) && config.sampleSpacing > 0);
    for (float limit : {config.maxHorizontalGap, config.maxClimb, config.maxDrop})
        require(std::isfinite(limit) && limit >= 0);
}

std::unordered_map<dtPolyRef, IslandId> indexTopology(const TopologyArtifact& topology, const Cancel& cancel) {
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

void validateAnchor(const Anchor& anchor, const std::unordered_map<dtPolyRef, IslandId>& index) {
    const auto it = index.find(anchor.polygon);
    require(finite(anchor.position) && it != index.end() && it->second == anchor.island);
}

bool eligible(const Anchor& from, const Anchor& to, const DiscoveryConfig& config) {
    const double dx = double(to.position.x) - from.position.x;
    const double dz = double(to.position.z) - from.position.z;
    const double dy = double(to.position.y) - from.position.y;
    return std::hypot(dx, dz) <= config.maxHorizontalGap &&
        dy <= config.maxClimb && -dy <= config.maxDrop;
}

auto anchorKey(const Anchor& anchor) {
    return std::make_tuple(anchor.island, anchor.polygon,
        anchor.position.x, anchor.position.y, anchor.position.z);
}

auto crossingKey(const Crossing& crossing) {
    return std::make_pair(anchorKey(crossing.a), anchorKey(crossing.b));
}

using CrossingKey = decltype(crossingKey(Crossing{}));

bool sameDirection(const DirectionResult& a, const DirectionResult& b) {
    return a.geometricallyEligible == b.geometricallyEligible && a.policyAllowed == b.policyAllowed &&
        a.validation.state == b.validation.state && a.validation.reason == b.validation.reason;
}

void countDirection(const DirectionResult& direction, StageStats& stats) {
    if (!direction.geometricallyEligible || !direction.policyAllowed) return;
    switch (direction.validation.state) {
    case ValidationState::Valid: ++stats.validDirections; break;
    case ValidationState::Invalid: ++stats.invalidDirections; break;
    case ValidationState::Unknown: ++stats.unknownDirections; break;
    }
}

DirectionResult evaluate(const Anchor& from, const Anchor& to, const BuildIdentity& identity,
    const DiscoveryConfig& config, bool allowed, const ValidationOptions& options, StageStats& stats) {
    DirectionResult result;
    result.geometricallyEligible = eligible(from, to, config);
    result.policyAllowed = allowed;
    if (result.geometricallyEligible && allowed && options.validator) {
        checkpoint(options.canceled);
        ++stats.validatorCalls;
        result.validation = options.validator({from, to, identity});
        require(validState(result.validation.state));
        checkpoint(options.canceled);
    }
    countDirection(result, stats);
    return result;
}

void validateDirection(const DirectionResult& direction, bool expectedEligibility, bool hasValidator) {
    require(direction.geometricallyEligible == expectedEligibility);
    require(validState(direction.validation.state));
    if (!hasValidator || !direction.geometricallyEligible || !direction.policyAllowed)
        require(direction.validation.state == ValidationState::Unknown && direction.validation.reason == 0);
}

bool usable(const DirectionResult& direction, CompilePolicy policy) {
    return direction.geometricallyEligible && direction.policyAllowed &&
        (direction.validation.state == ValidationState::Valid ||
            (policy == CompilePolicy::GeometricOnly && direction.validation.state == ValidationState::Unknown));
}

} // namespace

StageResult<CrossingArtifact> validateCrossings(const TopologyArtifact& topology,
    const DiscoveryConfig& config, const std::vector<CrossingCandidate>& candidates,
    const ValidationOptions& options) {
    StageResult<CrossingArtifact> result;
    try {
        checkpoint(options.canceled);
        validateConfig(config);
        const auto index = indexTopology(topology, options.canceled);
        if (config.maxCandidates && candidates.size() > config.maxCandidates)
            throw Abort{StageStatus::BudgetExceeded};
        CrossingArtifact artifact;
        artifact.topology = topology;
        artifact.discovery = config;
        artifact.validatorSupplied = bool(options.validator);
        artifact.customOutboundPolicy = bool(options.outboundPolicy);
        std::vector<bool> allowed(topology.islandCount, true);
        for (std::size_t i = 0; i < allowed.size(); ++i) {
            checkpoint(options.canceled);
            if (options.outboundPolicy) allowed[i] = options.outboundPolicy(static_cast<IslandId>(i));
        }
        std::map<CrossingKey, Crossing> unique;
        for (const auto& candidate : candidates) {
            checkpoint(options.canceled);
            ++result.stats.candidatesVisited;
            validateAnchor(candidate.a, index);
            validateAnchor(candidate.b, index);
            require(candidate.a.island != candidate.b.island);
            Crossing crossing{candidate.a, candidate.b, {}, {}};
            normalizeZero(crossing.a.position);
            normalizeZero(crossing.b.position);
            if (crossing.b.island < crossing.a.island) std::swap(crossing.a, crossing.b);
            const auto key = crossingKey(crossing);
            if (unique.find(key) != unique.end()) {
                ++result.stats.exactDuplicates;
                continue;
            }
            crossing.ab = evaluate(crossing.a, crossing.b, topology.identity, config,
                allowed[crossing.a.island], options, result.stats);
            crossing.ba = evaluate(crossing.b, crossing.a, topology.identity, config,
                allowed[crossing.b.island], options, result.stats);
            unique.emplace(key, std::move(crossing));
        }
        for (auto& entry : unique) {
            checkpoint(options.canceled);
            artifact.crossings.push_back(std::move(entry.second));
        }
        checkpoint(options.canceled);
        result.value.emplace(std::move(artifact));
        result.status = StageStatus::Success;
    } catch (const Abort& error) { result.status = error.status; }
    catch (const std::bad_alloc&) { result.status = StageStatus::OutOfMemory; }
    catch (...) { result.status = StageStatus::CallbackFailed; }
    return result;
}

CompileResult compileGraph(const CrossingArtifact& artifact, const CompileOptions& options) {
    CompileResult result;
    try {
        checkpoint(options.canceled);
        validateConfig(artifact.discovery);
        require(options.policy == CompilePolicy::GeometricOnly || options.policy == CompilePolicy::ValidatedOnly);
        require(options.policy != CompilePolicy::ValidatedOnly || artifact.validatorSupplied);
        auto graph = std::make_shared<CompiledGraph>();
        graph->polygonIslands_ = indexTopology(artifact.topology, options.canceled);
        graph->identity_ = artifact.topology.identity;
        graph->discovery_ = artifact.discovery;
        graph->policy_ = options.policy;
        graph->customPolygonPolicy_ = artifact.topology.customPolygonPolicy;
        graph->validatorSupplied_ = artifact.validatorSupplied;
        graph->customOutboundPolicy_ = artifact.customOutboundPolicy;
        const auto& id = graph->identity_;
        graph->persistentReuseEligible_ = id.mesh && id.movementProfile &&
            (!artifact.topology.customPolygonPolicy || id.polygonPolicy) &&
            (!artifact.customOutboundPolicy || id.outboundPolicy) &&
            (!artifact.validatorSupplied || (id.validator && id.environment));
        std::map<CrossingKey, const Crossing*> unique;
        std::vector<int> outbound(artifact.topology.islandCount, -1);
        for (const auto& crossing : artifact.crossings) {
            checkpoint(options.canceled);
            ++result.stats.candidatesVisited;
            validateAnchor(crossing.a, graph->polygonIslands_);
            validateAnchor(crossing.b, graph->polygonIslands_);
            require(crossing.a.island < crossing.b.island);
            validateDirection(crossing.ab, eligible(crossing.a, crossing.b, artifact.discovery), artifact.validatorSupplied);
            validateDirection(crossing.ba, eligible(crossing.b, crossing.a, artifact.discovery), artifact.validatorSupplied);
            if (!artifact.customOutboundPolicy) require(crossing.ab.policyAllowed && crossing.ba.policyAllowed);
            for (const auto endpoint : {std::make_pair(crossing.a.island, crossing.ab.policyAllowed),
                     std::make_pair(crossing.b.island, crossing.ba.policyAllowed)}) {
                auto& permission = outbound[endpoint.first];
                require(permission == -1 || permission == int(endpoint.second));
                permission = int(endpoint.second);
            }
            const auto inserted = unique.emplace(crossingKey(crossing), &crossing);
            if (!inserted.second) {
                // Conflicting duplicate validation must not manufacture reverse permission.
                require(sameDirection(inserted.first->second->ab, crossing.ab) &&
                    sameDirection(inserted.first->second->ba, crossing.ba));
                ++result.stats.exactDuplicates;
            }
        }
        graph->offsets_.resize(artifact.topology.islandCount + 1, 0);
        for (const auto& entry : unique) {
            checkpoint(options.canceled);
            const auto& crossing = *entry.second;
            countDirection(crossing.ab, result.stats);
            countDirection(crossing.ba, result.stats);
            const bool ab = usable(crossing.ab, options.policy);
            const bool ba = usable(crossing.ba, options.policy);
            if (!ab && !ba) continue;
            graph->crossings_.push_back({crossing, ab, ba});
            if (ab) ++graph->offsets_[crossing.a.island + 1];
            if (ba) ++graph->offsets_[crossing.b.island + 1];
        }
        for (std::size_t i = 1; i < graph->offsets_.size(); ++i) {
            checkpoint(options.canceled);
            graph->offsets_[i] += graph->offsets_[i - 1];
        }
        graph->traversals_.resize(graph->offsets_.back());
        auto next = graph->offsets_;
        for (std::size_t i = 0; i < graph->crossings_.size(); ++i) {
            checkpoint(options.canceled);
            const auto& compiled = graph->crossings_[i];
            if (compiled.traversableAB) graph->traversals_[next[compiled.crossing.a.island]++] = {i, false};
            if (compiled.traversableBA) graph->traversals_[next[compiled.crossing.b.island]++] = {i, true};
        }
        checkpoint(options.canceled);
        result.stats.compiledCrossings = graph->crossings_.size();
        result.stats.compiledDirections = graph->traversals_.size();
        result.value.emplace(std::move(graph));
        result.status = StageStatus::Success;
    } catch (const Abort& error) { result.status = error.status; }
    catch (const std::bad_alloc&) { result.status = StageStatus::OutOfMemory; }
    catch (...) { result.status = StageStatus::CallbackFailed; }
    return result;
}

} // namespace detour_island_graph::v2
