#pragma once

#include "IslandGraph.h"

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

class IslandGraphSerializer {
public:
    static constexpr std::uint32_t Magic = 0x31474944U; // "DIG1"
    static constexpr std::uint32_t Version = 2;

    static SerializationStatus write(std::ostream& stream, const IslandGraph& graph);
    static SerializationResult read(std::istream& stream);
};

} // namespace detour_island_graph
