#pragma once

#include <DetourNavMesh.h>

#include <cstdint>
#include <optional>
#include <type_traits>
#include <unordered_map>
#include <utility>
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

template <
    typename T,
    typename = decltype(
        static_cast<float>(std::declval<const T&>().x),
        static_cast<float>(std::declval<const T&>().y),
        static_cast<float>(std::declval<const T&>().z))>
Vec3 makeVec3(const T& value) {
    return {
        static_cast<float>(value.x),
        static_cast<float>(value.y),
        static_cast<float>(value.z)};
}

struct Link {
    IslandId fromIsland = 0;
    IslandId toIsland = 0;
    Vec3 start;
    Vec3 end;
    float horizontalDistance = 0.0f;
    float verticalDistance = 0.0f;
};

struct Edge {
    IslandId islandA = 0;
    IslandId islandB = 0;
    Vec3 pointA;
    Vec3 pointB;
    float horizontalDistance = 0.0f;
    float verticalDeltaAB = 0.0f;
    bool traversableAB = false;
    bool traversableBA = false;
};

inline bool connectsIsland(const Edge& edge, IslandId island) noexcept {
    return edge.islandA == island || edge.islandB == island;
}

inline bool canTraverseFrom(const Edge& edge, IslandId island) noexcept {
    if (edge.islandA == island) {
        return edge.traversableAB;
    }
    if (edge.islandB == island) {
        return edge.traversableBA;
    }
    return false;
}

inline std::optional<IslandId> otherIsland(const Edge& edge, IslandId island) noexcept {
    if (edge.islandA == island) {
        return edge.islandB;
    }
    if (edge.islandB == island) {
        return edge.islandA;
    }
    return std::nullopt;
}

inline std::optional<Link> makeTraversalLink(const Edge& edge, IslandId fromIsland) {
    if (edge.islandA == fromIsland && edge.traversableAB) {
        return std::optional<Link>(Link{
            edge.islandA,
            edge.islandB,
            edge.pointA,
            edge.pointB,
            edge.horizontalDistance,
            edge.verticalDeltaAB});
    }
    if (edge.islandB == fromIsland && edge.traversableBA) {
        return std::optional<Link>(Link{
            edge.islandB,
            edge.islandA,
            edge.pointB,
            edge.pointA,
            edge.horizontalDistance,
            -edge.verticalDeltaAB});
    }
    return std::nullopt;
}

struct Island {
    IslandId id = 0;
    std::vector<dtPolyRef> polygons;
    std::vector<std::uint32_t> edgeIndices;
    Vec3 center;
    Vec3 boundsMin;
    Vec3 boundsMax;
    float massScore = 0.0f;
    bool suppressed = false;
};

class IslandGraph {
public:
    IslandGraph() = default;
    explicit IslandGraph(std::vector<Island> islands);
    IslandGraph(std::vector<Island> islands, std::vector<Edge> edges);

    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] const std::vector<Island>& islands() const noexcept;
    [[nodiscard]] const std::vector<Edge>& edges() const noexcept;
    [[nodiscard]] const Island* findIsland(IslandId id) const noexcept;
    [[nodiscard]] std::optional<IslandId> findIslandForPolygon(dtPolyRef polygon) const;

private:
    friend class IslandGraphBuilder;
    friend struct detail::IslandGraphAccess;

    void rebuildPolygonLookup();
    void rebuildAdjacency();

    std::vector<Island> m_islands;
    std::vector<Edge> m_edges;
    std::unordered_map<dtPolyRef, IslandId> m_polygonToIsland;
};

} // namespace detour_island_graph
