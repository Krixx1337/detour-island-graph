#include <detour_island_graph/v2/Build.h>

#include <chrono>
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

} // namespace detour_island_graph::v2
