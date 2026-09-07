#pragma once

#include <DetourNavMesh.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace detour_island_graph::v2 {

using IslandId = std::uint32_t;
using Cancel = std::function<bool()>;

struct Point {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct Anchor {
    IslandId island = 0;
    dtPolyRef polygon = 0;
    Point position;
};

// IDs describe immutable inputs, not pointer addresses. Zero means unversioned.
struct BuildIdentity {
    std::uint64_t mesh = 0;
    std::uint64_t polygonPolicy = 0;
    std::uint64_t outboundPolicy = 0;
    std::uint64_t movementProfile = 0;
    std::uint64_t validator = 0;
    std::uint64_t environment = 0;
    double unitsPerMeter = 1;
};

struct BuildInput {
    // Caller owns the mesh and freezes it, plus all callback captures, for the call.
    const dtNavMesh* navMesh = nullptr;
    BuildIdentity identity;
    std::function<bool(dtPolyRef, const dtMeshTile&, const dtPoly&)> polygonFilter;
    Cancel canceled;
};

struct DiscoveryConfig {
    float sampleSpacing = 0; // Required, in navmesh units; unrelated to reach.
    float maxHorizontalGap = 0;
    float maxClimb = 0;
    float maxDrop = 0;
    std::size_t maxSamples = 0; // Unique boundary samples; zero means uncapped.
    std::size_t maxCandidates = 0; // Raw candidates, before exact deduplication.
};

struct PolygonIsland {
    dtPolyRef polygon = 0;
    IslandId island = 0;
};

struct TopologyArtifact {
    BuildIdentity identity;
    bool customPolygonPolicy = false;
    std::size_t islandCount = 0; // Dense IDs [0, islandCount); no empty islands.
    std::vector<PolygonIsland> polygons;
};

struct BoundaryInterval {
    IslandId island = 0;
    dtPolyRef polygon = 0;
    unsigned char edge = 0;
    double begin = 0; // Parameters along the owning polygon edge, in [0, 1].
    double end = 1;
    Point start;
    Point finish;
};

struct SamplingArtifact {
    TopologyArtifact topology;
    float sampleSpacing = 0;
    std::vector<BoundaryInterval> intervals;
    std::vector<Anchor> samples;
};

struct CrossingCandidate {
    Anchor a;
    Anchor b;
};

enum class ValidationState : std::uint8_t { Unknown, Valid, Invalid };

struct ValidationResult {
    ValidationState state = ValidationState::Unknown;
    std::uint32_t reason = 0; // Caller-defined; zero means unspecified.
};

struct DirectionResult {
    bool geometricallyEligible = false;
    bool policyAllowed = false;
    ValidationResult validation;
};

struct Crossing {
    Anchor a; // Canonical order: a.island < b.island, never execution order.
    Anchor b;
    DirectionResult ab;
    DirectionResult ba;
};

struct ValidationRequest {
    Anchor from;
    Anchor to;
    BuildIdentity identity;
};

struct ValidationOptions {
    // Pure, deterministic callbacks over frozen captures. Exceptions abort the stage.
    std::function<bool(IslandId)> outboundPolicy;
    std::function<ValidationResult(const ValidationRequest&)> validator;
    Cancel canceled;
};

struct CrossingArtifact {
    TopologyArtifact topology;
    DiscoveryConfig discovery;
    bool validatorSupplied = false;
    bool customOutboundPolicy = false;
    std::vector<Crossing> crossings;
};

enum class CompilePolicy : std::uint8_t { GeometricOnly, ValidatedOnly };

struct CompileOptions {
    CompilePolicy policy = CompilePolicy::GeometricOnly;
    Cancel canceled;
};

enum class StageStatus : std::uint8_t {
    Success, InvalidInput, BudgetExceeded, Canceled, CallbackFailed, OutOfMemory
};

struct StageStats {
    std::size_t groundPolygonsVisited = 0;
    std::size_t eligiblePolygons = 0;
    std::size_t islands = 0;
    std::size_t boundaryIntervals = 0;
    std::size_t sampleAttempts = 0;
    std::size_t sampleDuplicates = 0;
    std::size_t samples = 0;
    std::size_t discoveryQueries = 0;
    std::size_t nearbyPolygons = 0;
    std::size_t projections = 0;
    std::size_t projectionFailures = 0;
    std::size_t candidatesVisited = 0;
    std::size_t exactDuplicates = 0;
    std::size_t validatorCalls = 0;
    std::size_t validDirections = 0;
    std::size_t invalidDirections = 0;
    std::size_t unknownDirections = 0;
    std::size_t compiledCrossings = 0;
    std::size_t compiledDirections = 0;
};

template <class T> struct StageResult {
    StageStatus status = StageStatus::InvalidInput;
    StageStats stats;
    std::optional<T> value; // Engaged only after complete success, including empty output.
};

struct CompiledCrossing {
    Crossing crossing;
    bool traversableAB = false;
    bool traversableBA = false;
};

struct Traversal {
    std::size_t crossing = 0;
    bool reverse = false;
};

class CompiledGraph;
using CompileResult = StageResult<std::shared_ptr<const CompiledGraph>>;

class CompiledGraph {
public:
    const BuildIdentity& identity() const noexcept { return identity_; }
    const DiscoveryConfig& discovery() const noexcept { return discovery_; }
    CompilePolicy policy() const noexcept { return policy_; }
    const std::vector<CompiledCrossing>& crossings() const noexcept { return crossings_; }
    const std::vector<Traversal>& traversals() const noexcept { return traversals_; }
    // Outgoing traversals for island i occupy [offsets[i], offsets[i+1]).
    const std::vector<std::size_t>& offsets() const noexcept { return offsets_; }
    const std::unordered_map<dtPolyRef, IslandId>& polygonIslands() const noexcept { return polygonIslands_; }
    bool persistentReuseEligible() const noexcept { return persistentReuseEligible_; }

private:
    friend CompileResult compileGraph(const CrossingArtifact&, const CompileOptions&);
    BuildIdentity identity_;
    DiscoveryConfig discovery_;
    CompilePolicy policy_ = CompilePolicy::GeometricOnly;
    bool persistentReuseEligible_ = false;
    std::vector<CompiledCrossing> crossings_;
    std::vector<Traversal> traversals_;
    std::vector<std::size_t> offsets_;
    std::unordered_map<dtPolyRef, IslandId> polygonIslands_;
};

// First build stage. The snapshot must be complete, valid, immutable Detour data.
// Only ground polygons participate, even with a custom filter. Eligible native
// neighbors must be reciprocal; one-way ground adjacency is rejected, not merged.
// Samples follow coarse polygon edges, not detail-mesh height or collision clearance.
// Spacing bounds 3D arclength in exact arithmetic, subject to float representation.
// Collapsed adjacent samples fail instead of silently losing requested resolution.
// Island IDs and duplicate ownership follow (tile x, y, layer, polygon index),
// independent of tile allocation order. Polygon refs remain snapshot-specific.
StageResult<SamplingArtifact> extractAndSample(
    const BuildInput& input, const DiscoveryConfig& config);

// Candidate discovery from boundary samples. Uses the collector queryPolygons
// overload so dense stacked geometry cannot truncate silently; the fixed-size
// overload is never used. Only emits different-island pairs within independent
// horizontal/climb/drop limits. Raw output with no exact deduplication;
// validateCrossings merges exact duplicates. Enforces maxCandidates while
// generating; exceeding it returns BudgetExceeded with no output.
// Cancellation is checked per sample and per nearby polygon.
StageResult<std::vector<CrossingCandidate>> discoverCandidates(
    const SamplingArtifact& sampling, const dtNavMesh& navMesh,
    const DiscoveryConfig& config, const Cancel& canceled = {});

// Entry point for already-discovered anchored candidates.
// Exact duplicates require identical anchors, including polygon refs. Distinct approaches survive.
StageResult<CrossingArtifact> validateCrossings(
    const TopologyArtifact& topology, const DiscoveryConfig& config,
    const std::vector<CrossingCandidate>& candidates, const ValidationOptions& options = {});

// Recompiles policy without invoking validators. Rejects malformed or conflicting artifacts.
CompileResult compileGraph(const CrossingArtifact& artifact, const CompileOptions& options = {});

using DiscoveryResult = StageResult<std::vector<CrossingCandidate>>;

struct StageTimings {
    double samplingMs = 0;
    double discoveryMs = 0;
    double validationMs = 0;
    double compilationMs = 0;
    double totalMs = 0;
};

struct PipelineResult {
    StageStatus status = StageStatus::InvalidInput;
    StageTimings timings;
    StageResult<SamplingArtifact> sampling;
    DiscoveryResult discovery;
    StageResult<CrossingArtifact> validation;
    CompileResult compilation;
};

// Convenience wrapper over the four stages with one consistent DiscoveryConfig.
// Sampling and discovery use input.canceled; validation and compilation use
// their own options' canceled callbacks. Each stage's full StageResult is
// preserved for testing, timing, and reuse; stages after the first failure
// are not executed and keep their default result (status InvalidInput, no
// value), so check per-stage statuses in order. Wall-clock timings cover each
// attempted stage plus the total.
PipelineResult buildGraph(const BuildInput& input, const DiscoveryConfig& config,
    const ValidationOptions& validation = {}, const CompileOptions& compilation = {});

} // namespace detour_island_graph::v2
