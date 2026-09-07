#pragma once

#include <detour_island_graph/v2/Build.h>

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <memory>

namespace detour_island_graph::v2 {

enum class SerializationStatus : std::uint8_t {
    Success,
    IoError,
    InvalidMagic,
    UnsupportedVersion,
    MalformedData,
    Canceled,
    CallbackFailed,
    OutOfMemory
};

struct SerializationResult {
    std::shared_ptr<const CompiledGraph> graph;
    SerializationStatus status = SerializationStatus::MalformedData;
};

struct DeserializationLimits {
    std::uint32_t maxIslandCount = 1'000'000U;
    std::uint32_t maxElementsPerVector = 16'000'000U;
    std::size_t maxAllocationBytes = 256U * 1024U * 1024U;
};

struct DecodeOptions {
    DeserializationLimits limits;
    Cancel canceled;
};

// Native persistence for V2 compiled graphs. Adjacency is rebuilt on decode
// through compileGraph, so stored crossings are revalidated (counts,
// references, finite geometry, directional states) and the rebuilt graph is
// accepted only when every stored crossing survives compilation. Identity,
// discovery settings, and compilation policy round-trip so hosts can compare
// cache identity before reuse. V1 blobs are rejected by magic. Byte order is
// little-endian with IEEE-754 float/double bits; streams must be opened in
// binary mode. Package version and V1 contracts are untouched until host
// migration; this format is V2-only.
class GraphSerializer {
public:
    static constexpr std::uint32_t Magic = 0x32474944U; // "DIG2"
    // Format version, independent of the library version.
    static constexpr std::uint32_t FormatVersion = 1;

    [[nodiscard]] static SerializationStatus write(std::ostream& stream, const CompiledGraph& graph);
    [[nodiscard]] static SerializationResult read(
        std::istream& stream, const DecodeOptions& options = {});
};

} // namespace detour_island_graph::v2
