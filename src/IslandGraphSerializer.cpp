#include <detour_island_graph/IslandGraphSerializer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <istream>
#include <limits>
#include <new>
#include <ostream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace detour_island_graph {
namespace {

static_assert(std::numeric_limits<float>::is_iec559, "IEEE-754 float representation required");

bool consumeAllocationBudget(std::size_t& remainingBytes, std::size_t count, std::size_t elementBytes) {
    if (elementBytes != 0 && count > remainingBytes / elementBytes) {
        return false;
    }
    remainingBytes -= count * elementBytes;
    return true;
}

template <typename T>
bool writeUnsigned(std::ostream& stream, T value) {
    static_assert(std::is_unsigned<T>::value, "unsigned integer required");
    for (std::size_t byte = 0; byte < sizeof(T); ++byte) {
        stream.put(static_cast<char>((value >> (byte * 8U)) & static_cast<T>(0xffU)));
    }
    return stream.good();
}

template <typename T>
bool readUnsigned(std::istream& stream, T& value) {
    static_assert(std::is_unsigned<T>::value, "unsigned integer required");
    value = 0;
    for (std::size_t byte = 0; byte < sizeof(T); ++byte) {
        const int input = stream.get();
        if (input == std::char_traits<char>::eof()) {
            return false;
        }
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
    if (!readUnsigned(stream, bits)) {
        return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
}

bool writeVec3(std::ostream& stream, const Vec3& value) {
    return writeFloat(stream, value.x) &&
        writeFloat(stream, value.y) &&
        writeFloat(stream, value.z);
}

bool readVec3(std::istream& stream, Vec3& value) {
    return readFloat(stream, value.x) &&
        readFloat(stream, value.y) &&
        readFloat(stream, value.z);
}

bool isFinite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool isFinite(const Edge& edge) {
    return isFinite(edge.pointA) &&
        isFinite(edge.pointB) &&
        std::isfinite(edge.horizontalDistance) &&
        std::isfinite(edge.verticalDeltaAB);
}

bool nearlyEqual(float lhs, float rhs) {
    const float scale = (std::max)({1.0f, std::abs(lhs), std::abs(rhs)});
    return std::abs(lhs - rhs) <= 0.0001f * scale;
}

bool hasOrderedBounds(const Island& island) {
    return island.boundsMin.x <= island.boundsMax.x &&
        island.boundsMin.y <= island.boundsMax.y &&
        island.boundsMin.z <= island.boundsMax.z;
}

bool hasConsistentGeometry(const Edge& edge) {
    const float deltaX = edge.pointB.x - edge.pointA.x;
    const float deltaZ = edge.pointB.z - edge.pointA.z;
    const float expectedHorizontalDistance = std::sqrt((deltaX * deltaX) + (deltaZ * deltaZ));
    const float expectedVerticalDelta = edge.pointB.y - edge.pointA.y;
    return edge.horizontalDistance >= 0.0f &&
        std::isfinite(expectedHorizontalDistance) &&
        std::isfinite(expectedVerticalDelta) &&
        nearlyEqual(edge.horizontalDistance, expectedHorizontalDistance) &&
        nearlyEqual(edge.verticalDeltaAB, expectedVerticalDelta);
}

bool writeEdge(std::ostream& stream, const Edge& edge) {
    return writeUnsigned(stream, edge.islandA) &&
        writeUnsigned(stream, edge.islandB) &&
        writeVec3(stream, edge.pointA) &&
        writeVec3(stream, edge.pointB) &&
        writeFloat(stream, edge.horizontalDistance) &&
        writeFloat(stream, edge.verticalDeltaAB) &&
        writeUnsigned(stream, static_cast<std::uint8_t>(edge.traversableAB ? 1 : 0)) &&
        writeUnsigned(stream, static_cast<std::uint8_t>(edge.traversableBA ? 1 : 0));
}

bool readEdge(std::istream& stream, Edge& edge) {
    std::uint8_t traversableAB = 0;
    std::uint8_t traversableBA = 0;
    if (!readUnsigned(stream, edge.islandA) ||
        !readUnsigned(stream, edge.islandB) ||
        !readVec3(stream, edge.pointA) ||
        !readVec3(stream, edge.pointB) ||
        !readFloat(stream, edge.horizontalDistance) ||
        !readFloat(stream, edge.verticalDeltaAB) ||
        !readUnsigned(stream, traversableAB) ||
        !readUnsigned(stream, traversableBA)) {
        return false;
    }
    if (traversableAB > 1 || traversableBA > 1) {
        return false;
    }
    edge.traversableAB = traversableAB == 1;
    edge.traversableBA = traversableBA == 1;
    return true;
}

bool writeCount(std::ostream& stream, std::size_t count) {
    return count <= (std::numeric_limits<std::uint32_t>::max)() &&
        writeUnsigned(stream, static_cast<std::uint32_t>(count));
}

bool readCount(std::istream& stream, std::uint32_t maximum, std::uint32_t& count) {
    return readUnsigned(stream, count) && count <= maximum;
}

SerializationResult malformed(const char* message) {
    SerializationResult result;
    result.status = SerializationStatus::MalformedData;
    result.message = message;
    return result;
}

} // namespace

SerializationStatus IslandGraphSerializer::write(std::ostream& stream, const IslandGraph& graph) {
    if (!writeUnsigned(stream, Magic) ||
        !writeUnsigned(stream, FormatVersion) ||
        !writeCount(stream, graph.islands().size()) ||
        !writeCount(stream, graph.edges().size())) {
        return SerializationStatus::IoError;
    }

    for (const Edge& edge : graph.edges()) {
        if (!writeEdge(stream, edge)) {
            return SerializationStatus::IoError;
        }
    }

    for (const Island& island : graph.islands()) {
        if (!writeUnsigned(stream, island.id) ||
            !writeVec3(stream, island.center) ||
            !writeVec3(stream, island.boundsMin) ||
            !writeVec3(stream, island.boundsMax) ||
            !writeFloat(stream, island.massScore) ||
            !writeUnsigned(stream, static_cast<std::uint8_t>(island.suppressed ? 1 : 0)) ||
            !writeCount(stream, island.polygons.size())) {
            return SerializationStatus::IoError;
        }
        for (dtPolyRef polygon : island.polygons) {
            if (!writeUnsigned(stream, static_cast<std::uint64_t>(polygon))) {
                return SerializationStatus::IoError;
            }
        }
        if (!writeCount(stream, island.edgeIndices.size())) {
            return SerializationStatus::IoError;
        }
        for (std::uint32_t edgeIndex : island.edgeIndices) {
            if (!writeUnsigned(stream, edgeIndex)) {
                return SerializationStatus::IoError;
            }
        }
    }

    return stream.good() ? SerializationStatus::Success : SerializationStatus::IoError;
}

SerializationResult IslandGraphSerializer::read(
    std::istream& stream,
    const DeserializationLimits& limits) {
    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    if (!readUnsigned(stream, magic) || !readUnsigned(stream, version)) {
        return malformed("Serialized graph header is truncated.");
    }
    if (magic != Magic) {
        SerializationResult result;
        result.status = SerializationStatus::InvalidMagic;
        result.message = "Serialized graph magic does not match.";
        return result;
    }
    if (version != FormatVersion) {
        SerializationResult result;
        result.status = SerializationStatus::UnsupportedVersion;
        result.message = "Serialized graph version is not supported.";
        return result;
    }

    std::uint32_t islandCount = 0;
    std::uint32_t edgeCount = 0;
    if (!readCount(stream, limits.maxIslandCount, islandCount) ||
        !readCount(stream, limits.maxElementsPerVector, edgeCount)) {
        return malformed("Serialized graph header counts are invalid.");
    }

    std::size_t remainingAllocationBytes = limits.maxAllocationBytes;
    if (!consumeAllocationBudget(remainingAllocationBytes, islandCount, sizeof(Island)) ||
        !consumeAllocationBudget(remainingAllocationBytes, edgeCount, sizeof(Edge))) {
        return malformed("Serialized graph exceeds the allocation budget.");
    }

    try {
        std::vector<Edge> edges(edgeCount);
        for (Edge& edge : edges) {
            if (!readEdge(stream, edge) ||
                edge.islandA >= islandCount ||
                edge.islandB >= islandCount ||
                edge.islandA == edge.islandB ||
                (!edge.traversableAB && !edge.traversableBA) ||
                !isFinite(edge) ||
                !hasConsistentGeometry(edge)) {
                return malformed("Serialized edge data is invalid.");
            }
        }
        std::vector<std::vector<std::uint32_t>> expectedAdjacency(islandCount);
        for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
            const Edge& edge = edges[edgeIndex];
            expectedAdjacency[edge.islandA].push_back(static_cast<std::uint32_t>(edgeIndex));
            expectedAdjacency[edge.islandB].push_back(static_cast<std::uint32_t>(edgeIndex));
        }

        std::vector<Island> islands(islandCount);
        std::unordered_map<dtPolyRef, IslandId> polygonOwners;
        for (std::uint32_t islandIndex = 0; islandIndex < islandCount; ++islandIndex) {
            Island& island = islands[islandIndex];
            std::uint8_t suppressed = 0;
            if (!readUnsigned(stream, island.id) ||
                island.id != islandIndex ||
                !readVec3(stream, island.center) ||
                !readVec3(stream, island.boundsMin) ||
                !readVec3(stream, island.boundsMax) ||
                !isFinite(island.center) ||
                !isFinite(island.boundsMin) ||
                !isFinite(island.boundsMax) ||
                !hasOrderedBounds(island) ||
                !readFloat(stream, island.massScore) ||
                !std::isfinite(island.massScore) ||
                island.massScore < 0.0f ||
                island.massScore > 1.0f ||
                !readUnsigned(stream, suppressed) ||
                suppressed > 1) {
                return malformed("Serialized island data is invalid.");
            }
            island.suppressed = suppressed != 0;

            std::uint32_t polygonCount = 0;
            constexpr std::size_t polygonAllocationEstimate =
                sizeof(dtPolyRef) + sizeof(std::pair<const dtPolyRef, IslandId>) + (2 * sizeof(void*));
            if (!readCount(stream, limits.maxElementsPerVector, polygonCount) ||
                !consumeAllocationBudget(remainingAllocationBytes, polygonCount, polygonAllocationEstimate)) {
                return malformed("Serialized polygon count exceeds the allocation budget.");
            }
            island.polygons.resize(polygonCount);
            for (dtPolyRef& polygon : island.polygons) {
                std::uint64_t serializedPolygon = 0;
                if (!readUnsigned(stream, serializedPolygon) ||
                    serializedPolygon == 0 ||
                    serializedPolygon > static_cast<std::uint64_t>((std::numeric_limits<dtPolyRef>::max)())) {
                    return malformed("Serialized polygon reference is invalid.");
                }
                polygon = static_cast<dtPolyRef>(serializedPolygon);
                if (!polygonOwners.emplace(polygon, island.id).second) {
                    return malformed("Serialized polygon reference belongs to multiple islands.");
                }
            }

            std::uint32_t edgeIndexCount = 0;
            if (!readCount(stream, limits.maxElementsPerVector, edgeIndexCount) ||
                !consumeAllocationBudget(remainingAllocationBytes, edgeIndexCount, sizeof(std::uint32_t))) {
                return malformed("Serialized adjacency count exceeds the allocation budget.");
            }
            if (edgeIndexCount != expectedAdjacency[islandIndex].size()) {
                return malformed("Serialized island adjacency does not match graph edges.");
            }
            for (std::size_t adjacencyIndex = 0; adjacencyIndex < edgeIndexCount; ++adjacencyIndex) {
                std::uint32_t edgeIndex = 0;
                if (!readUnsigned(stream, edgeIndex) ||
                    edgeIndex != expectedAdjacency[islandIndex][adjacencyIndex]) {
                    return malformed("Serialized island adjacency is invalid.");
                }
            }
        }
        expectedAdjacency.clear();

        SerializationResult result;
        result.graph = IslandGraph(std::move(islands), std::move(edges));
        return result;
    } catch (const std::bad_alloc&) {
        return malformed("Serialized graph allocation failed.");
    }
}

} // namespace detour_island_graph
