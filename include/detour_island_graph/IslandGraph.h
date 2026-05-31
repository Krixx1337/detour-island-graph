#pragma once

#include <detour_island_graph/Version.h>

#include <DetourNavMesh.h>

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace detour_island_graph {

using IslandId = std::uint32_t;

namespace detail {
struct IslandGraphAccess;
}

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Construct from Detour float[3]
    explicit Vec3(const float* p) : x(p[0]), y(p[1]), z(p[2]) {}

    // Export to Detour float[3]
    void copyTo(float* p) const { p[0] = x; p[1] = y; p[2] = z; }

};

struct Link {
    IslandId fromIsland = 0;
    IslandId toIsland = 0;
    Vec3 start;
    Vec3 end;
    float horizontalDistance = 0.0f;
    float verticalDistance = 0.0f;
};

struct Island {
    IslandId id = 0;
    std::vector<dtPolyRef> polygons;
    std::vector<Link> outgoingLinks;
    Vec3 center;
    Vec3 boundsMin;
    Vec3 boundsMax;
    float massScore = 0.0f;
};

class IslandGraph {
public:
    IslandGraph() = default;
    explicit IslandGraph(std::vector<Island> islands);

    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] const std::vector<Island>& islands() const noexcept;
    [[nodiscard]] const Island* findIsland(IslandId id) const noexcept;
    [[nodiscard]] std::optional<IslandId> findIslandForPolygon(dtPolyRef polygon) const;

private:
    friend class IslandGraphBuilder;
    friend struct detail::IslandGraphAccess;

    void rebuildPolygonLookup();

    std::vector<Island> m_islands;
    std::unordered_map<dtPolyRef, IslandId> m_polygonToIsland;
};

} // namespace detour_island_graph
