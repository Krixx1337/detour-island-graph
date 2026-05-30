#include <detour_island_graph/IslandGraph.h>

#include <cassert>
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

std::vector<Island>& IslandGraph::mutableIslands() noexcept {
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

void IslandGraph::addLink(IslandId from, const Link& link) {
    assert(from < m_islands.size());
    assert(link.fromIsland == from);
    m_islands[from].outgoingLinks.push_back(link);
}

void IslandGraph::clearLinks(IslandId island) {
    assert(island < m_islands.size());
    m_islands[island].outgoingLinks.clear();
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
