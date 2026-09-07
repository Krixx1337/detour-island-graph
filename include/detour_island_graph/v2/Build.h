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

struct IslandMetrics {
    std::size_t polygonCount = 0;
    // Sum of coarse polygon triangle-fan 3D areas, in squared navmesh units.
    // Does not include detail-mesh relief or imply clearance/playability.
    double surfaceArea = 0;
    Point boundsMin;
    Point boundsMax;
};

// Unexplored is outside current coverage, never a claim of unreachability.
enum class DomainState : std::uint8_t { Included, Excluded, Unexplored };

struct IslandDomain {
    DomainState state = DomainState::Included;
    std::uint32_t reason = 0; // Host-defined evidence/policy reason.
};

struct BuildCoverage {
    bool seeded = false;
    bool complete = true; // Seeded builds publish only after frontier exhaustion.
    std::uint64_t seedIdentity = 0; // Versioned host seed semantics; zero disables reuse.
    std::vector<Anchor> seeds; // Checked, projected anchors in canonical order.
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
    std::uint64_t domainPolicy = 0;
};

struct BuildInput {
    // Caller owns the mesh and freezes it, plus all callback captures, for the call.
    const dtNavMesh* navMesh = nullptr;
    BuildIdentity identity;
    std::function<bool(dtPolyRef, const dtMeshTile&, const dtPoly&)> polygonFilter;
    Cancel canceled;
    // Whole-island decisions preserve native ownership and metrics. Polygon
    // exclusions that split connectivity must instead use polygonFilter.
    std::function<IslandDomain(IslandId, const IslandMetrics&)> islandPolicy;
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
    // Empty metrics means unavailable for trusted external producers; otherwise
    // exactly islandCount records. Mesh extraction always supplies metrics.
    std::vector<IslandMetrics> metrics;
    // Empty means exhaustive inclusion. Otherwise exactly islandCount records.
    std::vector<IslandDomain> domain;
    bool customDomainPolicy = false;
    BuildCoverage coverage;
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

struct TopologyExtractionArtifact {
    TopologyArtifact topology;
    std::vector<BoundaryInterval> intervals;
};

struct SamplingOptions {
    // Unset selects all Included islands; an empty vector selects none. Order
    // and duplicates do not affect output. Excluded/Unexplored islands never
    // sample, even when explicitly listed. Selection retains target ownership.
    std::optional<std::vector<IslandId>> islands;
    Cancel canceled;
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
    std::size_t includedIslands = 0;
    std::size_t excludedIslands = 0;
    std::size_t unexploredIslands = 0;
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
    bool customPolygonPolicy() const noexcept { return customPolygonPolicy_; }
    bool validatorSupplied() const noexcept { return validatorSupplied_; }
    bool customOutboundPolicy() const noexcept { return customOutboundPolicy_; }
    const std::vector<IslandMetrics>& metrics() const noexcept { return metrics_; }
    const std::vector<IslandDomain>& domain() const noexcept { return domain_; }
    bool customDomainPolicy() const noexcept { return customDomainPolicy_; }
    const BuildCoverage& coverage() const noexcept { return coverage_; }
    bool includes(IslandId island) const noexcept {
        return island < domain_.size() && domain_[island].state == DomainState::Included;
    }

private:
    friend CompileResult compileGraph(const CrossingArtifact&, const CompileOptions&);
    BuildIdentity identity_;
    DiscoveryConfig discovery_;
    CompilePolicy policy_ = CompilePolicy::GeometricOnly;
    bool persistentReuseEligible_ = false;
    bool customPolygonPolicy_ = false;
    bool validatorSupplied_ = false;
    bool customOutboundPolicy_ = false;
    bool customDomainPolicy_ = false;
    BuildCoverage coverage_;
    std::vector<IslandMetrics> metrics_;
    std::vector<IslandDomain> domain_;
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

// Extract native ownership, coarse metrics and exposed intervals once, without
// generating samples. The same frozen artifact can serve multiple selections.
StageResult<TopologyExtractionArtifact> extractTopology(const BuildInput& input);

// Artifacts are trusted producer inputs tied to the original mesh snapshot.
// Samples and intervals include only selected islands; topology retains all
// eligible polygons so discovery can find destinations outside the selection.
StageResult<SamplingArtifact> sampleBoundaries(const TopologyExtractionArtifact& topology,
    const DiscoveryConfig& config, const SamplingOptions& options = {});

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

struct SeededBuildOptions {
    // Explicit polygon refs avoid ambiguous nearest-layer selection. Every seed
    // is required, must belong to its stated eligible island, and project over
    // that polygon within this 3D tolerance in navmesh units.
    std::vector<Anchor> seeds;
    float projectionTolerance = 0;
    std::uint64_t seedIdentity = 0;
};

// Requires a validator. Only Valid, permitted outgoing directions expand reach.
// Exclusions persist; other islands begin Unexplored. Publishes ValidatedOnly
// after exhaustion, never partial output. Sample/candidate caps span all batches.
// Intermediate batches are discarded; exact crossing evidence is retained.
CompileResult buildSeededGraph(const BuildInput& input, const DiscoveryConfig& config,
    const SeededBuildOptions& seeds, const ValidationOptions& validation);

} // namespace detour_island_graph::v2
