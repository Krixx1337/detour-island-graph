#pragma once

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
};

class IslandGraph {
public:
    IslandGraph() = default;
    explicit IslandGraph(std::vector<Island> islands);

    bool empty() const noexcept;
    const std::vector<Island>& islands() const noexcept;
    const Island* findIsland(IslandId id) const noexcept;
    std::optional<IslandId> findIslandForPolygon(dtPolyRef polygon) const;

private:
    friend class IslandGraphBuilder;
    friend struct detail::IslandGraphAccess;

    void rebuildPolygonLookup();

    std::vector<Island> m_islands;
    std::unordered_map<dtPolyRef, IslandId> m_polygonToIsland;
};

} // namespace detour_island_graph
