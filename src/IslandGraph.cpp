#include <detour_island_graph/IslandGraph.h>

#include <utility>

namespace detour_island_graph {

IslandGraph::IslandGraph(std::vector<Island> islands)
    : m_islands(std::move(islands)) {
    rebuildPolygonLookup();
    rebuildAdjacency();
}

IslandGraph::IslandGraph(std::vector<Island> islands, std::vector<Edge> edges)
    : m_islands(std::move(islands)),
      m_edges(std::move(edges)) {
    rebuildPolygonLookup();
    rebuildAdjacency();
}

bool IslandGraph::empty() const noexcept {
    return m_islands.empty();
}

const std::vector<Island>& IslandGraph::islands() const noexcept {
    return m_islands;
}

const std::vector<Edge>& IslandGraph::edges() const noexcept {
    return m_edges;
}

const Island* IslandGraph::findIsland(IslandId id) const noexcept {
    if (id >= m_islands.size() || m_islands[id].id != id) {
        return nullptr;
    }
    return &m_islands[id];
}

std::optional<IslandId> IslandGraph::findIslandForPolygon(dtPolyRef polygon) const {
    const auto it = m_polygonToIsland.find(polygon);
    if (it == m_polygonToIsland.end()) {
        return std::nullopt;
    }
    return it->second;
}

void IslandGraph::rebuildPolygonLookup() {
    m_polygonToIsland.clear();
    for (const Island& island : m_islands) {
        for (dtPolyRef polygon : island.polygons) {
            m_polygonToIsland.emplace(polygon, island.id);
        }
    }
}

void IslandGraph::rebuildAdjacency() {
    for (Island& island : m_islands) {
        island.edgeIndices.clear();
    }
    for (std::size_t edgeIndex = 0; edgeIndex < m_edges.size(); ++edgeIndex) {
        const Edge& edge = m_edges[edgeIndex];
        if (edge.islandA < m_islands.size()) {
            m_islands[edge.islandA].edgeIndices.push_back(static_cast<std::uint32_t>(edgeIndex));
        }
        if (edge.islandB < m_islands.size() && edge.islandB != edge.islandA) {
            m_islands[edge.islandB].edgeIndices.push_back(static_cast<std::uint32_t>(edgeIndex));
        }
    }
}

} // namespace detour_island_graph
