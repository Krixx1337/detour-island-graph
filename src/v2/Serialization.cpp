#include <detour_island_graph/v2/Serialization.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <istream>
#include <limits>
#include <new>
#include <ostream>
#include <type_traits>
#include <utility>
#include <vector>

namespace detour_island_graph::v2 {
namespace {

static_assert(std::numeric_limits<float>::is_iec559, "IEEE-754 float representation required");
static_assert(std::numeric_limits<double>::is_iec559, "IEEE-754 double representation required");

struct Abort { SerializationStatus status; };

void checkpoint(const Cancel& canceled) {
    if (!canceled) return;
    bool abort = false;
    try {
        abort = canceled();
    } catch (...) {
        throw Abort{SerializationStatus::CallbackFailed};
    }
    if (abort) throw Abort{SerializationStatus::Canceled};
}

bool consumeBudget(std::size_t& remaining, std::size_t count, std::size_t elementBytes) {
    if (elementBytes != 0 && count > remaining / elementBytes) return false;
    remaining -= count * elementBytes;
    return true;
}

template <typename T> bool writeUnsigned(std::ostream& stream, T value) {
    static_assert(std::is_unsigned<T>::value, "unsigned integer required");
    for (std::size_t byte = 0; byte < sizeof(T); ++byte)
        stream.put(static_cast<char>((value >> (byte * 8U)) & static_cast<T>(0xffU)));
    return stream.good();
}

template <typename T> bool readUnsigned(std::istream& stream, T& value) {
    static_assert(std::is_unsigned<T>::value, "unsigned integer required");
    value = 0;
    for (std::size_t byte = 0; byte < sizeof(T); ++byte) {
        const int input = stream.get();
        if (input == std::char_traits<char>::eof()) return false;
        value |= static_cast<T>(static_cast<unsigned char>(input)) << (byte * 8U);
    }
    return true;
}

bool writeFloat(std::ostream& stream, float value) {
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "32-bit float required");
    std::memcpy(&bits, &value, sizeof(bits));
    return writeUnsigned(stream, bits);
}

bool readFloat(std::istream& stream, float& value) {
    std::uint32_t bits = 0;
    if (!readUnsigned(stream, bits)) return false;
    std::memcpy(&value, &bits, sizeof(value));
    return true;
}

bool writeDouble(std::ostream& stream, double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "64-bit double required");
    std::memcpy(&bits, &value, sizeof(bits));
    return writeUnsigned(stream, bits);
}

bool readDouble(std::istream& stream, double& value) {
    std::uint64_t bits = 0;
    if (!readUnsigned(stream, bits)) return false;
    std::memcpy(&value, &bits, sizeof(value));
    return true;
}

bool writePoint(std::ostream& stream, Point value) {
    return writeFloat(stream, value.x) && writeFloat(stream, value.y) && writeFloat(stream, value.z);
}

bool readPoint(std::istream& stream, Point& value) {
    return readFloat(stream, value.x) && readFloat(stream, value.y) && readFloat(stream, value.z);
}

bool writeCount(std::ostream& stream, std::size_t count) {
    return count <= (std::numeric_limits<std::uint32_t>::max)() &&
        writeUnsigned(stream, static_cast<std::uint32_t>(count));
}

bool writeAnchor(std::ostream& stream, const Anchor& anchor) {
    return writeUnsigned(stream, anchor.island) &&
        writeUnsigned(stream, static_cast<std::uint64_t>(anchor.polygon)) &&
        writePoint(stream, anchor.position);
}

bool writeDirection(std::ostream& stream, const DirectionResult& direction) {
    return writeUnsigned(stream, static_cast<std::uint8_t>(direction.geometricallyEligible ? 1 : 0)) &&
        writeUnsigned(stream, static_cast<std::uint8_t>(direction.policyAllowed ? 1 : 0)) &&
        writeUnsigned(stream, static_cast<std::uint8_t>(direction.validation.state)) &&
        writeUnsigned(stream, direction.validation.reason);
}

bool writeCrossing(std::ostream& stream, const Crossing& crossing) {
    return writeAnchor(stream, crossing.a) && writeAnchor(stream, crossing.b) &&
        writeDirection(stream, crossing.ab) && writeDirection(stream, crossing.ba);
}

} // namespace

SerializationStatus GraphSerializer::write(std::ostream& stream, const CompiledGraph& graph) {
    const std::size_t islandCount =
        graph.offsets().empty() ? 0 : graph.offsets().size() - 1;
    std::vector<std::pair<dtPolyRef, IslandId>> polygons(
        graph.polygonIslands().begin(), graph.polygonIslands().end());
    std::sort(polygons.begin(), polygons.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    const auto& identity = graph.identity();
    const auto& discovery = graph.discovery();
    if (!writeUnsigned(stream, Magic) ||
        !writeUnsigned(stream, FormatVersion) ||
        !writeCount(stream, islandCount) ||
        !writeCount(stream, polygons.size()) ||
        !writeCount(stream, graph.crossings().size()) ||
        !writeUnsigned(stream, identity.mesh) ||
        !writeUnsigned(stream, identity.polygonPolicy) ||
        !writeUnsigned(stream, identity.outboundPolicy) ||
        !writeUnsigned(stream, identity.movementProfile) ||
        !writeUnsigned(stream, identity.validator) ||
        !writeUnsigned(stream, identity.environment) ||
        !writeDouble(stream, identity.unitsPerMeter) ||
        !writeUnsigned(stream, static_cast<std::uint8_t>(graph.customPolygonPolicy() ? 1 : 0)) ||
        !writeUnsigned(stream, static_cast<std::uint8_t>(graph.customOutboundPolicy() ? 1 : 0)) ||
        !writeUnsigned(stream, static_cast<std::uint8_t>(graph.validatorSupplied() ? 1 : 0)) ||
        !writeFloat(stream, discovery.sampleSpacing) ||
        !writeFloat(stream, discovery.maxHorizontalGap) ||
        !writeFloat(stream, discovery.maxClimb) ||
        !writeFloat(stream, discovery.maxDrop) ||
        !writeUnsigned(stream, static_cast<std::uint64_t>(discovery.maxSamples)) ||
        !writeUnsigned(stream, static_cast<std::uint64_t>(discovery.maxCandidates)) ||
        !writeUnsigned(stream, static_cast<std::uint8_t>(graph.policy()))) {
        return SerializationStatus::IoError;
    }
    for (const auto& entry : polygons) {
        if (!writeUnsigned(stream, static_cast<std::uint64_t>(entry.first)) ||
            !writeUnsigned(stream, entry.second)) {
            return SerializationStatus::IoError;
        }
    }
    for (const auto& compiled : graph.crossings()) {
        if (!writeCrossing(stream, compiled.crossing)) return SerializationStatus::IoError;
    }
    if (!writeUnsigned(stream, identity.domainPolicy) ||
        !writeUnsigned(stream, static_cast<std::uint8_t>(graph.customDomainPolicy())) ||
        !writeCount(stream, graph.metrics().size())) return SerializationStatus::IoError;
    for (const auto& metric : graph.metrics()) {
        if (!writeCount(stream, metric.polygonCount) || !writeDouble(stream, metric.surfaceArea) ||
            !writePoint(stream, metric.boundsMin) || !writePoint(stream, metric.boundsMax))
            return SerializationStatus::IoError;
    }
    for (const auto& decision : graph.domain()) {
        if (!writeUnsigned(stream, static_cast<std::uint8_t>(decision.state)) ||
            !writeUnsigned(stream, decision.reason)) return SerializationStatus::IoError;
    }
    const auto& coverage = graph.coverage();
    if (!writeUnsigned(stream, static_cast<std::uint8_t>(coverage.seeded)) ||
        !writeUnsigned(stream, static_cast<std::uint8_t>(coverage.complete)) ||
        !writeUnsigned(stream, coverage.seedIdentity) || !writeCount(stream, coverage.seeds.size()))
        return SerializationStatus::IoError;
    for (const auto& seed : coverage.seeds)
        if (!writeAnchor(stream, seed)) return SerializationStatus::IoError;
    return stream.good() ? SerializationStatus::Success : SerializationStatus::IoError;
}

SerializationResult GraphSerializer::read(std::istream& stream, const DecodeOptions& options) {
    SerializationResult result;
    try {
        std::uint32_t magic = 0, version = 0;
        if (!readUnsigned(stream, magic) || !readUnsigned(stream, version)) {
            result.status = SerializationStatus::MalformedData;
            return result;
        }
        if (magic != Magic) {
            result.status = SerializationStatus::InvalidMagic;
            return result;
        }
        if (version != FormatVersion) {
            result.status = SerializationStatus::UnsupportedVersion;
            return result;
        }
        const auto& limits = options.limits;
        std::uint32_t islandCount = 0, polygonCount = 0, crossingCount = 0;
        if (!readUnsigned(stream, islandCount) || islandCount > limits.maxIslandCount ||
            !readUnsigned(stream, polygonCount) || polygonCount > limits.maxElementsPerVector ||
            !readUnsigned(stream, crossingCount) || crossingCount > limits.maxElementsPerVector) {
            result.status = SerializationStatus::MalformedData;
            return result;
        }
        std::size_t remaining = limits.maxAllocationBytes;
        if (!consumeBudget(remaining, islandCount, sizeof(std::size_t)) ||
            !consumeBudget(remaining, polygonCount,
                sizeof(PolygonIsland) + sizeof(std::pair<const dtPolyRef, IslandId>)) ||
            !consumeBudget(remaining, crossingCount, sizeof(Crossing))) {
            result.status = SerializationStatus::MalformedData;
            return result;
        }

        CrossingArtifact artifact;
        auto& identity = artifact.topology.identity;
        std::uint8_t customPolygon = 0, customOutbound = 0, validatorSupplied = 0,
                     policyByte = 0;
        std::uint64_t maxSamples = 0, maxCandidates = 0;
        if (!readUnsigned(stream, identity.mesh) ||
            !readUnsigned(stream, identity.polygonPolicy) ||
            !readUnsigned(stream, identity.outboundPolicy) ||
            !readUnsigned(stream, identity.movementProfile) ||
            !readUnsigned(stream, identity.validator) ||
            !readUnsigned(stream, identity.environment) ||
            !readDouble(stream, identity.unitsPerMeter) ||
            !readUnsigned(stream, customPolygon) || customPolygon > 1 ||
            !readUnsigned(stream, customOutbound) || customOutbound > 1 ||
            !readUnsigned(stream, validatorSupplied) || validatorSupplied > 1 ||
            !readFloat(stream, artifact.discovery.sampleSpacing) ||
            !readFloat(stream, artifact.discovery.maxHorizontalGap) ||
            !readFloat(stream, artifact.discovery.maxClimb) ||
            !readFloat(stream, artifact.discovery.maxDrop) ||
            !readUnsigned(stream, maxSamples) ||
            !readUnsigned(stream, maxCandidates) ||
            !readUnsigned(stream, policyByte) || policyByte > 1) {
            result.status = SerializationStatus::MalformedData;
            return result;
        }
        if (maxSamples > (std::numeric_limits<std::size_t>::max)() ||
            maxCandidates > (std::numeric_limits<std::size_t>::max)()) {
            result.status = SerializationStatus::MalformedData;
            return result;
        }
        artifact.discovery.maxSamples = static_cast<std::size_t>(maxSamples);
        artifact.discovery.maxCandidates = static_cast<std::size_t>(maxCandidates);
        artifact.topology.customPolygonPolicy = customPolygon != 0;
        artifact.customOutboundPolicy = customOutbound != 0;
        artifact.validatorSupplied = validatorSupplied != 0;
        artifact.topology.islandCount = islandCount;

        artifact.topology.polygons.reserve(polygonCount);
        for (std::uint32_t i = 0; i < polygonCount; ++i) {
            checkpoint(options.canceled);
            std::uint64_t ref = 0;
            std::uint32_t island = 0;
            if (!readUnsigned(stream, ref) || !readUnsigned(stream, island)) {
                result.status = SerializationStatus::MalformedData;
                return result;
            }
            if (ref == 0 ||
                ref > static_cast<std::uint64_t>((std::numeric_limits<dtPolyRef>::max)())) {
                result.status = SerializationStatus::MalformedData;
                return result;
            }
            artifact.topology.polygons.push_back({static_cast<dtPolyRef>(ref), island});
        }
        artifact.crossings.reserve(crossingCount);
        for (std::uint32_t i = 0; i < crossingCount; ++i) {
            checkpoint(options.canceled);
            Crossing crossing;
            std::uint64_t polyA = 0, polyB = 0;
            std::uint8_t eligibleAB = 0, allowedAB = 0, stateAB = 0;
            std::uint8_t eligibleBA = 0, allowedBA = 0, stateBA = 0;
            if (!readUnsigned(stream, crossing.a.island) || !readUnsigned(stream, polyA) ||
                !readPoint(stream, crossing.a.position) ||
                !readUnsigned(stream, crossing.b.island) || !readUnsigned(stream, polyB) ||
                !readPoint(stream, crossing.b.position) ||
                !readUnsigned(stream, eligibleAB) || eligibleAB > 1 ||
                !readUnsigned(stream, allowedAB) || allowedAB > 1 ||
                !readUnsigned(stream, stateAB) || stateAB > 2 ||
                !readUnsigned(stream, crossing.ab.validation.reason) ||
                !readUnsigned(stream, eligibleBA) || eligibleBA > 1 ||
                !readUnsigned(stream, allowedBA) || allowedBA > 1 ||
                !readUnsigned(stream, stateBA) || stateBA > 2 ||
                !readUnsigned(stream, crossing.ba.validation.reason)) {
                result.status = SerializationStatus::MalformedData;
                return result;
            }
            if (polyA == 0 || polyA > static_cast<std::uint64_t>((std::numeric_limits<dtPolyRef>::max)()) ||
                polyB == 0 || polyB > static_cast<std::uint64_t>((std::numeric_limits<dtPolyRef>::max)())) {
                result.status = SerializationStatus::MalformedData;
                return result;
            }
            crossing.a.polygon = static_cast<dtPolyRef>(polyA);
            crossing.b.polygon = static_cast<dtPolyRef>(polyB);
            crossing.ab.geometricallyEligible = eligibleAB != 0;
            crossing.ab.policyAllowed = allowedAB != 0;
            crossing.ab.validation.state = static_cast<ValidationState>(stateAB);
            crossing.ba.geometricallyEligible = eligibleBA != 0;
            crossing.ba.policyAllowed = allowedBA != 0;
            crossing.ba.validation.state = static_cast<ValidationState>(stateBA);
            artifact.crossings.push_back(std::move(crossing));
        }

        std::uint8_t customDomain = 0;
        std::uint32_t metricCount = 0;
        if (!readUnsigned(stream, identity.domainPolicy) ||
            !readUnsigned(stream, customDomain) || customDomain > 1 ||
            !readUnsigned(stream, metricCount) || (metricCount != 0 && metricCount != islandCount) ||
            !consumeBudget(remaining, metricCount, 2 * sizeof(IslandMetrics)) ||
            !consumeBudget(remaining, islandCount, 2 * sizeof(IslandDomain))) {
            result.status = SerializationStatus::MalformedData;
            return result;
        }
        artifact.topology.customDomainPolicy = customDomain != 0;
        artifact.topology.metrics.resize(metricCount);
        for (auto& metric : artifact.topology.metrics) {
            checkpoint(options.canceled);
            std::uint32_t count = 0;
            if (!readUnsigned(stream, count) || !readDouble(stream, metric.surfaceArea) ||
                !readPoint(stream, metric.boundsMin) || !readPoint(stream, metric.boundsMax)) {
                result.status = SerializationStatus::MalformedData;
                return result;
            }
            metric.polygonCount = count;
        }
        artifact.topology.domain.resize(islandCount);
        for (auto& decision : artifact.topology.domain) {
            checkpoint(options.canceled);
            std::uint8_t state = 0;
            if (!readUnsigned(stream, state) || state > 2 || !readUnsigned(stream, decision.reason)) {
                result.status = SerializationStatus::MalformedData;
                return result;
            }
            decision.state = static_cast<DomainState>(state);
        }

        auto& coverage = artifact.topology.coverage;
        std::uint8_t seeded = 0, complete = 0;
        std::uint32_t seedCount = 0;
        if (!readUnsigned(stream, seeded) || seeded > 1 ||
            !readUnsigned(stream, complete) || complete != 1 ||
            !readUnsigned(stream, coverage.seedIdentity) || !readUnsigned(stream, seedCount) ||
            seedCount > limits.maxElementsPerVector ||
            !consumeBudget(remaining, seedCount, 2 * sizeof(Anchor))) {
            result.status = SerializationStatus::MalformedData;
            return result;
        }
        coverage.seeded = seeded != 0;
        coverage.complete = true;
        coverage.seeds.resize(seedCount);
        for (auto& seed : coverage.seeds) {
            checkpoint(options.canceled);
            std::uint64_t ref = 0;
            if (!readUnsigned(stream, seed.island) || !readUnsigned(stream, ref) ||
                ref == 0 || ref > static_cast<std::uint64_t>((std::numeric_limits<dtPolyRef>::max)()) ||
                !readPoint(stream, seed.position)) {
                result.status = SerializationStatus::MalformedData;
                return result;
            }
            seed.polygon = static_cast<dtPolyRef>(ref);
        }

        // Revalidation and adjacency rebuild go through the compiler: stored
        // crossings must reproduce the same compiled graph under the stored
        // policy, otherwise the blob is inconsistent.
        CompileOptions compileOptions;
        compileOptions.policy = static_cast<CompilePolicy>(policyByte);
        compileOptions.canceled = options.canceled;
        CompileResult compiled = compileGraph(artifact, compileOptions);
        if (compiled.status == StageStatus::Canceled) {
            result.status = SerializationStatus::Canceled;
            return result;
        }
        if (compiled.status == StageStatus::OutOfMemory) {
            result.status = SerializationStatus::OutOfMemory;
            return result;
        }
        if (compiled.status == StageStatus::CallbackFailed) {
            result.status = SerializationStatus::CallbackFailed;
            return result;
        }
        if (compiled.status != StageStatus::Success || !compiled.value ||
            compiled.stats.compiledCrossings != crossingCount) {
            result.status = SerializationStatus::MalformedData;
            return result;
        }
        result.graph = *compiled.value;
        result.status = SerializationStatus::Success;
    } catch (const Abort& error) {
        result.status = error.status;
    } catch (const std::bad_alloc&) {
        result.status = SerializationStatus::OutOfMemory;
    } catch (...) {
        result.status = SerializationStatus::MalformedData;
    }
    return result;
}

} // namespace detour_island_graph::v2
