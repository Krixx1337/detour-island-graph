#pragma once

#include "IslandGraph.h"

#include <cstdint>
#include <iosfwd>
#include <string>

namespace detour_island_graph {

enum class SerializationStatus {
    Success,
    IoError,
    InvalidMagic,
    UnsupportedVersion,
    MalformedData
};

struct SerializationResult {
    IslandGraph graph;
    SerializationStatus status = SerializationStatus::Success;
    std::string message;

    explicit operator bool() const noexcept {
        return status == SerializationStatus::Success;
    }
};

struct DeserializationLimits {
    std::uint32_t maxIslandCount = 1'000'000U;
    std::uint32_t maxElementsPerVector = 16'000'000U;
    std::size_t maxAllocationBytes = 256U * 1024U * 1024U;
};

class IslandGraphSerializer {
public:
    static constexpr std::uint32_t Magic = 0x31474944U; // "DIG1"
    // File format version. This is intentionally independent from the library version.
    static constexpr std::uint32_t FormatVersion = 4;

    [[nodiscard]] static SerializationStatus write(std::ostream& stream, const IslandGraph& graph);
    [[nodiscard]] static SerializationResult read(
        std::istream& stream,
        const DeserializationLimits& limits = {});
};

} // namespace detour_island_graph
