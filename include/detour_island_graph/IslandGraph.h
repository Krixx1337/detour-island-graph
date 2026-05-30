#pragma once

#include <DetourNavMesh.h>

#include <cstdint>
#include <initializer_list>
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

    Vec3& operator=(std::initializer_list<float> ilist) {
        auto it = ilist.begin();
        x = *it++;
        y = *it++;
        z = *it++;
        return *this;
    }
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

    bool empty() const noexcept;
    const std::vector<Island>& islands() const noexcept;
    std::vector<Island>& mutableIslands() noexcept;
    const Island* findIsland(IslandId id) const noexcept;
    std::optional<IslandId> findIslandForPolygon(dtPolyRef polygon) const;

    // Allow users to inject custom designer links (e.g. ziplines, jump pads)
    void addLink(IslandId from, const Link& link);

    // Allow users to clear links (e.g. when a bridge is destroyed)
    void clearLinks(IslandId island);

private:
    friend class IslandGraphBuilder;
    friend struct detail::IslandGraphAccess;

    void rebuildPolygonLookup();

    std::vector<Island> m_islands;
    std::unordered_map<dtPolyRef, IslandId> m_polygonToIsland;
};

} // namespace detour_island_graph
