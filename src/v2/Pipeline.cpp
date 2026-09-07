#include <detour_island_graph/v2/Build.h>

#include <chrono>
#include <DetourNavMeshQuery.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>
#include <new>

namespace detour_island_graph::v2 {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

} // namespace

PipelineResult buildGraph(const BuildInput& input, const DiscoveryConfig& config,
    const ValidationOptions& validation, const CompileOptions& compilation) {
    PipelineResult result;
    const auto totalStart = Clock::now();
    try {
        auto stageStart = Clock::now();
        result.sampling = extractAndSample(input, config);
        result.timings.samplingMs = elapsedMs(stageStart);
        if (result.sampling.status != StageStatus::Success || !result.sampling.value ||
            input.navMesh == nullptr) {
            result.status = result.sampling.status;
            result.timings.totalMs = elapsedMs(totalStart);
            return result;
        }
        stageStart = Clock::now();
        result.discovery =
            discoverCandidates(*result.sampling.value, *input.navMesh, config, input.canceled);
        result.timings.discoveryMs = elapsedMs(stageStart);
        if (result.discovery.status != StageStatus::Success || !result.discovery.value) {
            result.status = result.discovery.status;
            result.timings.totalMs = elapsedMs(totalStart);
            return result;
        }
        stageStart = Clock::now();
        result.validation = validateCrossings(
            result.sampling.value->topology, config, *result.discovery.value, validation);
        result.timings.validationMs = elapsedMs(stageStart);
        if (result.validation.status != StageStatus::Success || !result.validation.value) {
            result.status = result.validation.status;
            result.timings.totalMs = elapsedMs(totalStart);
            return result;
        }
        stageStart = Clock::now();
        result.compilation = compileGraph(*result.validation.value, compilation);
        result.timings.compilationMs = elapsedMs(stageStart);
        result.status = result.compilation.status;
    } catch (const std::bad_alloc&) {
        result.status = StageStatus::OutOfMemory;
    } catch (...) {
        result.status = StageStatus::CallbackFailed;
    }
    result.timings.totalMs = elapsedMs(totalStart);
    return result;
}

CompileResult buildSeededGraph(const BuildInput& input, const DiscoveryConfig& config,
    const SeededBuildOptions& seeds, const ValidationOptions& validation) {
    CompileResult result;
    struct Abort { StageStatus status; };
    try {
        const Cancel canceled = [&] {
            return (input.canceled && input.canceled()) ||
                (validation.canceled && validation.canceled());
        };
        const auto check = [&] { if (canceled()) throw Abort{StageStatus::Canceled}; };
        check();
        if (!validation.validator || seeds.seeds.empty() ||
            !std::isfinite(seeds.projectionTolerance) || seeds.projectionTolerance < 0)
            return result;
        BuildInput frozen = input;
        frozen.canceled = canceled;
        auto extracted = extractTopology(frozen);
        if (!extracted.value) { result.status = extracted.status; return result; }
        auto& extraction = *extracted.value;
        auto& topology = extraction.topology;
        topology.coverage = {true, false, seeds.seedIdentity, {}};
        for (auto& decision : topology.domain)
            if (decision.state != DomainState::Excluded) decision.state = DomainState::Unexplored;
        std::unordered_map<dtPolyRef, IslandId> ownership;
        for (const auto& polygon : topology.polygons) {
            check();
            ownership.emplace(polygon.polygon, polygon.island);
        }
        dtNavMeshQuery query;
        const auto initialized = query.init(input.navMesh, 256);
        if (dtStatusFailed(initialized)) {
            result.status = (initialized & DT_OUT_OF_MEMORY) ? StageStatus::OutOfMemory : StageStatus::InvalidInput;
            return result;
        }
        std::set<IslandId> pending;
        const auto anchorKey = [](const Anchor& a) {
            return std::make_tuple(a.island, a.polygon, a.position.x, a.position.y, a.position.z);
        };
        for (auto seed : seeds.seeds) {
            check();
            const auto found = ownership.find(seed.polygon);
            if (found == ownership.end() || found->second != seed.island ||
                topology.domain[seed.island].state == DomainState::Excluded ||
                !std::isfinite(seed.position.x) || !std::isfinite(seed.position.y) ||
                !std::isfinite(seed.position.z)) return result;
            const float position[] = {seed.position.x, seed.position.y, seed.position.z};
            float projected[3];
            bool over = false;
            if (dtStatusFailed(query.closestPointOnPoly(seed.polygon, position, projected, &over)) || !over ||
                !std::isfinite(projected[0]) || !std::isfinite(projected[1]) || !std::isfinite(projected[2]) ||
                std::hypot(double(projected[0]) - position[0], double(projected[1]) - position[1],
                    double(projected[2]) - position[2]) > seeds.projectionTolerance) return result;
            seed.position = {projected[0] == 0 ? 0 : projected[0], projected[1] == 0 ? 0 : projected[1],
                projected[2] == 0 ? 0 : projected[2]};
            topology.coverage.seeds.push_back(seed);
            topology.domain[seed.island].state = DomainState::Included;
            pending.insert(seed.island);
        }
        auto& checkedSeeds = topology.coverage.seeds;
        std::sort(checkedSeeds.begin(), checkedSeeds.end(), [&](const Anchor& a, const Anchor& b) {
            return anchorKey(a) < anchorKey(b);
        });
        checkedSeeds.erase(std::unique(checkedSeeds.begin(), checkedSeeds.end(), [&](const Anchor& a, const Anchor& b) {
            return anchorKey(a) == anchorKey(b);
        }), checkedSeeds.end());
        CrossingArtifact artifact;
        artifact.discovery = config;
        artifact.validatorSupplied = true;
        artifact.customOutboundPolicy = bool(validation.outboundPolicy);
        using Key = decltype(std::make_pair(anchorKey(Anchor{}), anchorKey(Anchor{})));
        std::set<Key> seen;
        ValidationOptions options = validation;
        options.canceled = canceled;
        while (!pending.empty()) {
            check();
            const auto island = *pending.begin();
            pending.erase(pending.begin());
            SamplingOptions samplingOptions;
            samplingOptions.islands = std::vector<IslandId>{island};
            samplingOptions.canceled = canceled;
            auto batchConfig = config;
            if (config.maxSamples)
                batchConfig.maxSamples = (std::max)(std::size_t{1}, config.maxSamples - result.stats.samples);
            auto sampled = sampleBoundaries(extraction, batchConfig, samplingOptions);
            result.stats.samples += sampled.stats.samples;
            if (!sampled.value) throw Abort{sampled.status};
            if (config.maxSamples && result.stats.samples > config.maxSamples)
                throw Abort{StageStatus::BudgetExceeded};
            if (config.maxCandidates)
                batchConfig.maxCandidates = (std::max)(std::size_t{1}, config.maxCandidates - result.stats.candidatesVisited);
            auto discovered = discoverCandidates(*sampled.value, *input.navMesh, batchConfig, canceled);
            result.stats.candidatesVisited += discovered.stats.candidatesVisited;
            if (!discovered.value) throw Abort{discovered.status};
            if (config.maxCandidates && result.stats.candidatesVisited > config.maxCandidates)
                throw Abort{StageStatus::BudgetExceeded};
            std::vector<CrossingCandidate> fresh;
            for (auto candidate : *discovered.value) {
                check();
                if (candidate.a.island > candidate.b.island) std::swap(candidate.a, candidate.b);
                if (seen.emplace(anchorKey(candidate.a), anchorKey(candidate.b)).second)
                    fresh.push_back(candidate);
                else ++result.stats.exactDuplicates;
            }
            auto validated = validateCrossings(topology, config, fresh, options);
            result.stats.validatorCalls += validated.stats.validatorCalls;
            if (!validated.value) throw Abort{validated.status};
            for (const auto& crossing : validated.value->crossings) {
                check();
                const auto activate = [&](const Anchor& from, const Anchor& to, const DirectionResult& direction) {
                    if (topology.domain[from.island].state == DomainState::Included &&
                        topology.domain[to.island].state == DomainState::Unexplored &&
                        direction.geometricallyEligible && direction.policyAllowed &&
                        direction.validation.state == ValidationState::Valid) {
                        topology.domain[to.island].state = DomainState::Included;
                        pending.insert(to.island);
                    }
                };
                activate(crossing.a, crossing.b, crossing.ab);
                activate(crossing.b, crossing.a, crossing.ba);
                artifact.crossings.push_back(crossing);
            }
        }
        check();
        topology.coverage.complete = true;
        artifact.topology = std::move(topology);
        auto compiled = compileGraph(artifact, {CompilePolicy::ValidatedOnly, canceled});
        compiled.stats.samples = result.stats.samples;
        compiled.stats.candidatesVisited = result.stats.candidatesVisited;
        compiled.stats.exactDuplicates += result.stats.exactDuplicates;
        compiled.stats.validatorCalls = result.stats.validatorCalls;
        return compiled;
    } catch (const Abort& error) { result.status = error.status; }
    catch (const std::bad_alloc&) { result.status = StageStatus::OutOfMemory; }
    catch (...) { result.status = StageStatus::CallbackFailed; }
    return result;
}

} // namespace detour_island_graph::v2
