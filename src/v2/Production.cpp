#include "BuildBudget.h"

#include <DetourNavMeshQuery.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <tuple>

namespace detour_island_graph::v2 {
namespace {
using detail::BuildAbort;
using Clock = std::chrono::steady_clock;

void require(bool condition) { if (!condition) throw BuildAbort{StageStatus::InvalidInput}; }
bool finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
void normalize(Point& p) {
    if (p.x == 0) p.x = 0;
    if (p.y == 0) p.y = 0;
    if (p.z == 0) p.z = 0;
}
auto pointKey(Point p) { return std::make_tuple(p.x, p.y, p.z); }
auto anchorKey(const Anchor& a) {
    return std::make_tuple(a.island, a.polygon, a.position.x, a.position.y, a.position.z);
}
using Key = decltype(std::make_pair(anchorKey(Anchor{}), anchorKey(Anchor{})));
using SampleKey = decltype(pointKey(Point{}));
bool eligible(const Anchor& a, const Anchor& b, const DiscoveryConfig& c) {
    const double dy = double(b.position.y) - a.position.y;
    return std::hypot(double(b.position.x) - a.position.x, double(b.position.z) - a.position.z)
        <= c.maxHorizontalGap && dy <= c.maxClimb && -dy <= c.maxDrop;
}

// Attribute nested sampling/discovery/validation work exclusively to its stage.
struct StageClock {
    double* current = nullptr;
    Clock::time_point since = Clock::now();
    void switchTo(double* next) {
        const auto now = Clock::now();
        if (current) *current += std::chrono::duration<double, std::milli>(now - since).count();
        since = now;
        current = next;
    }
};
struct TimedStage {
    StageClock& clock;
    double* previous;
    TimedStage(StageClock& c, double& target) : clock(c), previous(c.current) { clock.switchTo(&target); }
    ~TimedStage() { clock.switchTo(previous); }
};

class Collector final : public dtPolyQuery {
public:
    Collector(BuildVector<dtPolyRef>& output, const Cancel& cancel, StageStats& stats,
        const BuildLimits& limits, BudgetDiagnostics& diagnostics)
        : output_(output), cancel_(cancel), stats_(stats), limits_(limits), diagnostics_(diagnostics) {}
    void process(const dtMeshTile*, dtPoly**, dtPolyRef* refs, int count) override {
        if (status != StageStatus::Success) return;
        try {
            require(count >= 0);
            for (int i = 0; i < count; ++i) {
                detail::checkCancel(cancel_);
                detail::work();
                detail::limit(BudgetResource::NearbyRefsPerQuery, output_.size(), 1, limits_.maxNearbyRefsPerQuery);
                output_.push_back(refs[i]);
                ++stats_.nearbyPolygons;
                diagnostics_.peakNearbyRefs = (std::max)(diagnostics_.peakNearbyRefs, output_.size());
            }
        } catch (const BuildAbort& e) { status = e.status; }
        catch (const std::bad_alloc&) { status = StageStatus::OutOfMemory; }
        catch (...) { status = StageStatus::CallbackFailed; }
    }
    StageStatus status = StageStatus::Success;
private:
    BuildVector<dtPolyRef>& output_;
    const Cancel& cancel_;
    StageStats& stats_;
    const BuildLimits& limits_;
    BudgetDiagnostics& diagnostics_;
};

void validateOptions(const BuildInput& input, const DiscoveryConfig& config,
    const ProductionBuildOptions& options, const ValidationOptions& validation,
    const CompileOptions& compilation, const SeededBuildOptions* seeds) {
    const auto& l = options.limits;
    require(l.maxAllocationBytes && l.maxWorkUnits && l.maxTopologyPolygons && l.maxBoundaryIntervals &&
        l.maxNearbyRefsPerQuery && l.maxUniqueCrossings && l.maxCompiledCrossings && l.maxCompiledDirections &&
        options.sampleBatchSize && options.candidateBatchSize && config.maxSamples && config.maxCandidates);
    require(input.navMesh && std::isfinite(config.sampleSpacing) && config.sampleSpacing > 0);
    for (float x : {config.maxHorizontalGap, config.maxClimb, config.maxDrop}) require(std::isfinite(x) && x >= 0);
    require(compilation.policy == CompilePolicy::GeometricOnly || compilation.policy == CompilePolicy::ValidatedOnly);
    require(compilation.policy != CompilePolicy::ValidatedOnly || bool(validation.validator));
    if (seeds) require(validation.validator && !seeds->seeds.empty() &&
        std::isfinite(seeds->projectionTolerance) && seeds->projectionTolerance >= 0);
    require(std::isfinite(input.identity.unitsPerMeter) && input.identity.unitsPerMeter > 0);
}

class Builder {
public:
    Builder(const BuildInput& input, const DiscoveryConfig& config,
        const ProductionBuildOptions& options, const ValidationOptions& validation,
        const CompileOptions& compilation, ProductionBuildResult& result,
        const std::shared_ptr<detail::AllocationAccount>& account)
        : input_(input), config_(config), options_(options), validation_(validation),
          compilation_(compilation), result_(result), account_(account) {
        // Callback captures and std::function bookkeeping are outside the allocation contract.
        canceled_ = [&] {
            return (input_.canceled && input_.canceled()) ||
                (validation_.canceled && validation_.canceled()) ||
                (compilation_.canceled && compilation_.canceled());
        };
    }

    void run(const SeededBuildOptions* seeds) {
        check();
        {
            TimedStage timer(clock_, result_.timings.samplingMs);
            // Reuse topology implementation with the same budget and combined cancellation.
            BuildInput frozen = input_;
            frozen.canceled = canceled_;
            auto extracted = extractTopology(frozen);
            result_.stats = extracted.stats;
            if (!extracted.value) throw BuildAbort{extracted.status};
            extraction_ = std::move(*extracted.value);
            for (const auto& p : topology().polygons) {
                step();
                ownership_.emplace(p.polygon, p.island);
            }
            // Stable counting-sort by island retains original per-island edge order.
            offsets_.resize(topology().islandCount + 1, 0);
            for (const auto& interval : extraction_.intervals) { step(); ++offsets_[interval.island + 1]; }
            for (std::size_t i = 1; i < offsets_.size(); ++i) { step(); offsets_[i] += offsets_[i - 1]; }
            intervalOrder_.resize(extraction_.intervals.size());
            auto next = offsets_;
            for (std::size_t i = 0; i < extraction_.intervals.size(); ++i) {
                step();
                intervalOrder_[next[extraction_.intervals[i].island]++] = i;
            }
            allowed_.resize(topology().islandCount, true);
            for (std::size_t i = 0; i < allowed_.size(); ++i) {
                step();
                if (validation_.outboundPolicy)
                    allowed_[i] = detail::callback(validation_.outboundPolicy, static_cast<IslandId>(i));
                check();
            }
        }
        const auto initialized = query_.init(input_.navMesh, 256);
        if (dtStatusFailed(initialized)) throw BuildAbort{
            initialized & DT_OUT_OF_MEMORY ? StageStatus::OutOfMemory : StageStatus::InvalidInput};
        seeded_ = seeds != nullptr;
        if (seeds) initializeSeeds(*seeds);
        else for (std::size_t i = 0; i < topology().islandCount; ++i) {
            step();
            if (topology().domain[i].state == DomainState::Included) pending_.insert(static_cast<IslandId>(i));
        }
        while (!pending_.empty()) {
            check();
            const auto island = *pending_.begin();
            pending_.erase(pending_.begin());
            sampleIsland(island);
        }
        topology().coverage.complete = true;
        compile();
        check();
    }

private:
    TopologyArtifact& topology() { return extraction_.topology; }
    void check() { detail::checkCancel(canceled_); }
    void step() { check(); detail::work(); }
    void initializeSeeds(const SeededBuildOptions& seeds) {
        TimedStage timer(clock_, result_.timings.samplingMs);
        topology().coverage = {true, false, seeds.seedIdentity, {}};
        for (auto& d : topology().domain) { step(); if (d.state != DomainState::Excluded) d.state = DomainState::Unexplored; }
        for (auto seed : seeds.seeds) {
            step();
            const auto it = ownership_.find(seed.polygon);
            require(it != ownership_.end() && it->second == seed.island && finite(seed.position));
            require(topology().domain[seed.island].state != DomainState::Excluded);
            const float pos[] = {seed.position.x, seed.position.y, seed.position.z};
            float closest[3];
            bool over = false;
            require(dtStatusSucceed(query_.closestPointOnPoly(seed.polygon, pos, closest, &over)) && over);
            require(finite({closest[0], closest[1], closest[2]}) &&
                std::hypot(double(closest[0]) - pos[0], double(closest[1]) - pos[1], double(closest[2]) - pos[2])
                <= seeds.projectionTolerance);
            seed.position = {closest[0], closest[1], closest[2]};
            normalize(seed.position);
            topology().coverage.seeds.push_back(seed);
            topology().domain[seed.island].state = DomainState::Included;
            pending_.insert(seed.island);
        }
        auto& checked = topology().coverage.seeds;
        std::sort(checked.begin(), checked.end(), [&](const Anchor& a, const Anchor& b) {
            check(); return anchorKey(a) < anchorKey(b);
        });
        checked.erase(std::unique(checked.begin(), checked.end(), [&](const Anchor& a, const Anchor& b) {
            check(); return anchorKey(a) == anchorKey(b);
        }), checked.end());
    }
    void sampleIsland(IslandId island) {
        TimedStage timer(clock_, result_.timings.samplingMs);
        detail::Set<SampleKey> unique;
        for (auto j = offsets_[island]; j < offsets_[island + 1]; ++j) {
            step();
            const auto& interval = extraction_.intervals[intervalOrder_[j]];
            auto a = interval.start, b = interval.finish;
            if (pointKey(b) < pointKey(a)) std::swap(a, b);
            const double length = std::hypot(double(b.x) - a.x, double(b.y) - a.y, double(b.z) - a.z);
            require(length > 0 && std::isfinite(length));
            const double divisions = std::ceil(length / config_.sampleSpacing);
            require(std::isfinite(divisions));
            // Work limit rejects huge intervals before integer conversion/iteration.
            if (divisions >= double((std::numeric_limits<std::size_t>::max)() - 1)) {
                detail::limit(BudgetResource::WorkUnits, options_.limits.maxWorkUnits, 1, options_.limits.maxWorkUnits);
            }
            const auto segments = static_cast<std::size_t>(divisions);
            Point previous;
            for (std::size_t i = 0; i <= segments; ++i) {
                step();
                const double t = double(i) / double(segments);
                Point p = i == 0 ? a : i == segments ? b : Point{
                    float(double(a.x) + (double(b.x) - a.x) * t),
                    float(double(a.y) + (double(b.y) - a.y) * t),
                    float(double(a.z) + (double(b.z) - a.z) * t)};
                normalize(p);
                require(i == 0 || pointKey(previous) != pointKey(p));
                previous = p;
                ++result_.stats.sampleAttempts;
                const auto key = pointKey(p);
                const auto pos = unique.lower_bound(key);
                if (pos != unique.end() && *pos == key) { ++result_.stats.sampleDuplicates; continue; }
                detail::limit(BudgetResource::Samples, result_.stats.samples, 1, config_.maxSamples);
                unique.emplace_hint(pos, key);
                samples_.push_back({island, interval.polygon, p});
                ++result_.stats.samples;
                account_->diagnostics.peakSampleBatch = (std::max)(account_->diagnostics.peakSampleBatch, samples_.size());
                if (samples_.size() == options_.sampleBatchSize) discover();
            }
        }
        discover();
        validate(); // Frontier expansion occurs before choosing next island, regardless of batch sizes.
    }
    void discover() {
        TimedStage timer(clock_, result_.timings.discoveryMs);
        dtQueryFilter filter;
        for (const auto& sample : samples_) {
            step();
            const float center[] = {sample.position.x, sample.position.y, sample.position.z};
            const float vertical = (std::max)(config_.maxClimb, config_.maxDrop);
            const float extents[] = {config_.maxHorizontalGap, vertical, config_.maxHorizontalGap};
            nearby_.clear();
            Collector collector(nearby_, canceled_, result_.stats, options_.limits, account_->diagnostics);
            ++result_.stats.discoveryQueries;
            const auto status = query_.queryPolygons(center, extents, &filter, &collector);
            if (collector.status != StageStatus::Success) throw BuildAbort{collector.status};
            require(dtStatusSucceed(status));
            check();
            std::sort(nearby_.begin(), nearby_.end(), [&](dtPolyRef a, dtPolyRef b) { check(); return a < b; });
            nearby_.erase(std::unique(nearby_.begin(), nearby_.end()), nearby_.end());
            for (auto ref : nearby_) {
                step();
                if (!ref || ref == sample.polygon) continue;
                const auto found = ownership_.find(ref);
                if (found == ownership_.end() || found->second == sample.island ||
                    topology().domain[found->second].state == DomainState::Excluded) continue;
                float closest[3];
                bool over = false;
                if (dtStatusFailed(query_.closestPointOnPoly(ref, center, closest, &over))) {
                    ++result_.stats.projectionFailures;
                    continue;
                }
                Point p{closest[0], closest[1], closest[2]};
                require(finite(p));
                normalize(p);
                ++result_.stats.projections;
                CrossingCandidate candidate{sample, {found->second, ref, p}};
                if (!eligible(candidate.a, candidate.b, config_) && !eligible(candidate.b, candidate.a, config_)) continue;
                detail::limit(BudgetResource::Candidates, result_.stats.candidatesVisited, 1, config_.maxCandidates);
                candidates_.push_back(candidate);
                ++result_.stats.candidatesVisited;
                account_->diagnostics.peakCandidateBatch = (std::max)(account_->diagnostics.peakCandidateBatch, candidates_.size());
                if (candidates_.size() == options_.candidateBatchSize) validate();
            }
        }
        samples_.clear();
    }
    DirectionResult evaluate(const Anchor& from, const Anchor& to) {
        step();
        DirectionResult d;
        d.geometricallyEligible = eligible(from, to, config_);
        d.policyAllowed = allowed_[from.island];
        if (!d.geometricallyEligible || !d.policyAllowed) return d;
        if (validation_.validator) {
            ++result_.stats.validatorCalls;
            d.validation = detail::callback(validation_.validator, ValidationRequest{from, to, input_.identity});
            check();
        }
        switch (d.validation.state) {
        case ValidationState::Valid: ++result_.stats.validDirections; break;
        case ValidationState::Invalid: ++result_.stats.invalidDirections; break;
        case ValidationState::Unknown: ++result_.stats.unknownDirections; break;
        default: throw BuildAbort{StageStatus::InvalidInput};
        }
        return d;
    }
    void activate(const Anchor& from, const Anchor& to, const DirectionResult& d) {
        if (seeded_ && topology().domain[from.island].state == DomainState::Included &&
            topology().domain[to.island].state == DomainState::Unexplored &&
            d.geometricallyEligible && d.policyAllowed && d.validation.state == ValidationState::Valid) {
            topology().domain[to.island].state = DomainState::Included;
            pending_.insert(to.island);
        }
    }
    void validate() {
        TimedStage timer(clock_, result_.timings.validationMs);
        for (auto candidate : candidates_) {
            step();
            if (candidate.b.island < candidate.a.island) std::swap(candidate.a, candidate.b);
            const Key key{anchorKey(candidate.a), anchorKey(candidate.b)};
            if (crossings_.find(key) != crossings_.end()) { ++result_.stats.exactDuplicates; continue; }
            detail::limit(BudgetResource::UniqueCrossings, crossings_.size(), 1, options_.limits.maxUniqueCrossings);
            Crossing c{candidate.a, candidate.b, {}, {}};
            c.ab = evaluate(c.a, c.b);
            c.ba = evaluate(c.b, c.a);
            crossings_.emplace(key, c);
            account_->diagnostics.uniqueCrossings = crossings_.size();
            activate(c.a, c.b, c.ab);
            activate(c.b, c.a, c.ba);
        }
        candidates_.clear();
    }
    void compile() {
        TimedStage timer(clock_, result_.timings.compilationMs);
        // Drop query/sampling scratch before allocating final graph; account includes
        // the temporary crossing artifact and final graph while both are alive.
        BuildVector<Anchor>().swap(samples_);
        BuildVector<CrossingCandidate>().swap(candidates_);
        BuildVector<dtPolyRef>().swap(nearby_);
        BuildVector<BoundaryInterval>().swap(extraction_.intervals);
        BuildVector<std::size_t>().swap(offsets_);
        BuildVector<std::size_t>().swap(intervalOrder_);
        BuildUnorderedMap<dtPolyRef, IslandId>().swap(ownership_);
        BuildVector<bool>().swap(allowed_);
        CrossingArtifact artifact;
        artifact.topology = std::move(topology());
        artifact.discovery = config_;
        artifact.validatorSupplied = bool(validation_.validator);
        artifact.customOutboundPolicy = bool(validation_.outboundPolicy);
        artifact.crossings.reserve(crossings_.size());
        while (!crossings_.empty()) {
            step();
            artifact.crossings.push_back(crossings_.begin()->second);
            crossings_.erase(crossings_.begin());
        }
        auto compiled = compileGraph(artifact, {seeded_ ? CompilePolicy::ValidatedOnly : compilation_.policy, canceled_});
        result_.stats.compiledCrossings = compiled.stats.compiledCrossings;
        result_.stats.compiledDirections = compiled.stats.compiledDirections;
        if (!compiled.value) throw BuildAbort{compiled.status};
        result_.stats.includedIslands = compiled.stats.includedIslands;
        result_.stats.excludedIslands = compiled.stats.excludedIslands;
        result_.stats.unexploredIslands = compiled.stats.unexploredIslands;
        result_.graph = std::move(*compiled.value);
    }

    const BuildInput& input_;
    const DiscoveryConfig& config_;
    const ProductionBuildOptions& options_;
    const ValidationOptions& validation_;
    const CompileOptions& compilation_;
    ProductionBuildResult& result_;
    const std::shared_ptr<detail::AllocationAccount>& account_;
    Cancel canceled_;
    StageClock clock_;
    TopologyExtractionArtifact extraction_;
    BuildUnorderedMap<dtPolyRef, IslandId> ownership_;
    BuildVector<std::size_t> offsets_, intervalOrder_;
    BuildVector<bool> allowed_;
    BuildVector<Anchor> samples_;
    BuildVector<CrossingCandidate> candidates_;
    BuildVector<dtPolyRef> nearby_;
    detail::Set<IslandId> pending_;
    detail::Map<Key, Crossing> crossings_;
    dtNavMeshQuery query_;
    bool seeded_ = false;
};

ProductionBuildResult build(const BuildInput& input, const DiscoveryConfig& config,
    const ProductionBuildOptions& options, const ValidationOptions& validation,
    const CompileOptions& compilation, const SeededBuildOptions* seeds) {
    ProductionBuildResult result;
    const auto start = Clock::now();
    std::shared_ptr<detail::AllocationAccount> account;
    try {
        // Validate before empty debug containers allocate or callbacks run.
        validateOptions(input, config, options, validation, compilation, seeds);
        detail::checkCancel(input.canceled);
        detail::checkCancel(validation.canceled);
        detail::checkCancel(compilation.canceled);
        // Fixed allocator bookkeeping is excluded, as documented in the API.
        account = std::make_shared<detail::AllocationAccount>();
        account->limits = options.limits;
        detail::AccountScope scope(account);
        Builder builder(input, config, options, validation, compilation, result, account);
        builder.run(seeds);
        result.status = StageStatus::Success;
    } catch (const BuildAbort& e) { result.status = e.status; }
    catch (const std::bad_alloc&) { result.status = StageStatus::OutOfMemory; }
    catch (...) { result.status = StageStatus::CallbackFailed; }
    if (result.status != StageStatus::Success) result.graph.reset();
    if (account) result.budget = account->diagnostics;
    result.timings.totalMs = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    return result;
}
} // namespace

ProductionBuildResult buildGraphBounded(const BuildInput& input, const DiscoveryConfig& config,
    const ProductionBuildOptions& options, const ValidationOptions& validation, const CompileOptions& compilation) {
    return build(input, config, options, validation, compilation, nullptr);
}
ProductionBuildResult buildSeededGraphBounded(const BuildInput& input, const DiscoveryConfig& config,
    const SeededBuildOptions& seeds, const ProductionBuildOptions& options, const ValidationOptions& validation) {
    return build(input, config, options, validation, {CompilePolicy::ValidatedOnly, {}}, &seeds);
}
} // namespace detour_island_graph::v2
