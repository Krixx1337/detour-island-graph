#include <detour_island_graph/IslandGraph.h>

#include <utility>

namespace detour_island_graph {

IslandGraph::IslandGraph(std::vector<Island> islands)
    : m_islands(std::move(islands)) {
    rebuildPolygonLookup();
}

bool IslandGraph::empty() const noexcept {
    return m_islands.empty();
}

const std::vector<Island>& IslandGraph::islands() const noexcept {
    return m_islands;
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

} // namespace detour_island_graph
