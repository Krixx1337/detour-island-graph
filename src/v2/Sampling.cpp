#include <detour_island_graph/v2/Build.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <set>
#include <tuple>
#include <utility>

namespace detour_island_graph::v2 {
namespace {

struct Abort { StageStatus status; };

void require(bool condition) {
    if (!condition) throw Abort{StageStatus::InvalidInput};
}

void checkpoint(const Cancel& canceled) {
    if (canceled && canceled()) throw Abort{StageStatus::Canceled};
}

Point vertex(const dtMeshTile& tile, const dtPoly& polygon, unsigned char index) {
    require(polygon.verts[index] < tile.header->vertCount);
    const float* v = tile.verts + 3 * polygon.verts[index];
    require(std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]));
    return {v[0], v[1], v[2]};
}

Point interpolate(Point a, Point b, double t) {
    if (t == 0) return a;
    if (t == 1) return b;
    return {float(double(a.x) + (double(b.x) - a.x) * t),
        float(double(a.y) + (double(b.y) - a.y) * t),
        float(double(a.z) + (double(b.z) - a.z) * t)};
}

auto pointKey(Point p) { return std::make_tuple(p.x, p.y, p.z); }
using SampleKey = std::tuple<IslandId, float, float, float>;

struct Polygon {
    const dtMeshTile* tile;
    const dtPoly* poly;
    dtPolyRef ref;
    std::vector<std::size_t> neighbors;
};

using PolygonIndex = std::unordered_map<dtPolyRef, std::size_t>;

// Native off-mesh links never participate. Internal neis and external edge links
// describe walk adjacency; action links attached to a ground polygon do not.
template <class Visitor>
void visitNeighbors(const dtNavMesh& mesh, const Polygon& polygon, unsigned char edge,
    const PolygonIndex& index, const Cancel& canceled, Visitor visit) {
    const auto neighbor = polygon.poly->neis[edge];
    if (!neighbor) return;
    if (!(neighbor & DT_EXT_LINK)) {
        require(neighbor <= polygon.tile->header->polyCount);
        const auto found = index.find(mesh.getPolyRefBase(polygon.tile) | dtPolyRef(neighbor - 1));
        if (found != index.end()) visit(found->second, 0, 255);
        return;
    }
    std::size_t visited = 0;
    for (auto linkIndex = polygon.poly->firstLink; linkIndex != DT_NULL_LINK;) {
        checkpoint(canceled);
        require(linkIndex < static_cast<unsigned int>(polygon.tile->header->maxLinkCount));
        require(++visited <= static_cast<std::size_t>(polygon.tile->header->maxLinkCount));
        const auto& link = polygon.tile->links[linkIndex];
        if (link.edge == edge && link.ref) {
            const auto found = index.find(link.ref);
            if (found != index.end()) {
                require(link.bmin <= link.bmax);
                visit(found->second, int(link.bmin), int(link.bmax));
            }
        }
        linkIndex = link.next;
    }
}

void sampleInterval(const BoundaryInterval& interval, const DiscoveryConfig& config,
    const Cancel& canceled, SamplingArtifact& artifact, std::set<SampleKey>& unique,
    StageStats& stats) {
    Point a = interval.start, b = interval.finish;
    // Equal geometric segments generate equal positions regardless of edge winding.
    if (pointKey(b) < pointKey(a)) std::swap(a, b);
    const double length = std::hypot(double(b.x) - a.x, double(b.y) - a.y, double(b.z) - a.z);
    require(length > 0);
    const double divisions = std::ceil(length / config.sampleSpacing);
    require(std::isfinite(divisions));
    // Avoid undefined float-to-integer conversion and an overflowing endpoint loop.
    if (divisions >= double((std::numeric_limits<std::size_t>::max)() - 1))
        throw Abort{StageStatus::BudgetExceeded};
    const auto segments = static_cast<std::size_t>(divisions);
    Point previous;
    for (std::size_t i = 0; i <= segments; ++i) {
        checkpoint(canceled);
        Point p = interpolate(a, b, double(i) / double(segments));
        if (p.x == 0) p.x = 0;
        if (p.y == 0) p.y = 0;
        if (p.z == 0) p.z = 0;
        // A requested resolution below coordinate precision must not become
        // successful but coarser coverage through exact duplicate elimination.
        require(i == 0 || pointKey(p) != pointKey(previous));
        previous = p;
        ++stats.sampleAttempts;
        const SampleKey key{interval.island, p.x, p.y, p.z};
        const auto position = unique.lower_bound(key);
        if (position != unique.end() && *position == key) {
            ++stats.sampleDuplicates;
            continue;
        }
        if (config.maxSamples && stats.samples == config.maxSamples)
            throw Abort{StageStatus::BudgetExceeded};
        unique.emplace_hint(position, key);
        artifact.samples.push_back({interval.island, interval.polygon, p});
        ++stats.samples;
    }
}

} // namespace

StageResult<SamplingArtifact> extractAndSample(const BuildInput& input, const DiscoveryConfig& config) {
    StageResult<SamplingArtifact> result;
    try {
        checkpoint(input.canceled);
        require(input.navMesh != nullptr);
        require(std::isfinite(input.identity.unitsPerMeter) && input.identity.unitsPerMeter > 0);
        require(std::isfinite(config.sampleSpacing) && config.sampleSpacing > 0);
        for (float limit : {config.maxHorizontalGap, config.maxClimb, config.maxDrop})
            require(std::isfinite(limit) && limit >= 0);
        const auto& mesh = *input.navMesh;
        std::vector<const dtMeshTile*> tiles;
        for (int i = 0; i < mesh.getMaxTiles(); ++i) {
            checkpoint(input.canceled);
            const auto* tile = mesh.getTile(i);
            if (tile && tile->header) tiles.push_back(tile);
        }
        const auto tileKey = [](const dtMeshTile* tile) {
            return std::make_tuple(tile->header->x, tile->header->y, tile->header->layer);
        };
        std::sort(tiles.begin(), tiles.end(), [&](const auto* a, const auto* b) {
            checkpoint(input.canceled);
            return tileKey(a) < tileKey(b);
        });
        for (std::size_t i = 1; i < tiles.size(); ++i) require(tileKey(tiles[i - 1]) != tileKey(tiles[i]));

        std::vector<Polygon> polygons;
        PolygonIndex index;
        for (const auto* tile : tiles) {
            checkpoint(input.canceled);
            require(tile->header->polyCount >= 0 && tile->header->maxLinkCount >= 0);
            for (int i = 0; i < tile->header->polyCount; ++i) {
                checkpoint(input.canceled);
                const auto& poly = tile->polys[i];
                if (poly.getType() != DT_POLYTYPE_GROUND) continue;
                ++result.stats.groundPolygonsVisited;
                const auto ref = mesh.getPolyRefBase(tile) | dtPolyRef(i);
                if (input.polygonFilter && !input.polygonFilter(ref, *tile, poly)) continue;
                require(poly.vertCount >= 3 && poly.vertCount <= DT_VERTS_PER_POLYGON);
                for (unsigned char v = 0; v < poly.vertCount; ++v) vertex(*tile, poly, v);
                require(index.emplace(ref, polygons.size()).second);
                polygons.push_back({tile, &poly, ref, {}});
                ++result.stats.eligiblePolygons;
            }
        }
        checkpoint(input.canceled);
        for (auto& polygon : polygons) {
            for (unsigned char edge = 0; edge < polygon.poly->vertCount; ++edge) {
                checkpoint(input.canceled);
                visitNeighbors(mesh, polygon, edge, index, input.canceled,
                    [&](std::size_t neighbor, int, int) { polygon.neighbors.push_back(neighbor); });
            }
            std::sort(polygon.neighbors.begin(), polygon.neighbors.end());
            polygon.neighbors.erase(std::unique(polygon.neighbors.begin(), polygon.neighbors.end()),
                polygon.neighbors.end());
        }
        for (std::size_t i = 0; i < polygons.size(); ++i) {
            checkpoint(input.canceled);
            for (auto neighbor : polygons[i].neighbors) {
                const auto& reverse = polygons[neighbor].neighbors;
                // Weak connectivity would silently permit impossible reverse
                // on-island transfers. MVP supports reciprocal native ground only.
                require(std::binary_search(reverse.begin(), reverse.end(), i));
            }
        }

        SamplingArtifact artifact;
        artifact.topology.identity = input.identity;
        artifact.topology.customPolygonPolicy = bool(input.polygonFilter);
        artifact.sampleSpacing = config.sampleSpacing;
        std::vector<IslandId> owners(polygons.size(), (std::numeric_limits<IslandId>::max)());
        std::vector<std::size_t> queue;
        for (std::size_t i = 0; i < polygons.size(); ++i) {
            checkpoint(input.canceled);
            if (owners[i] != (std::numeric_limits<IslandId>::max)()) continue;
            require(artifact.topology.islandCount < (std::numeric_limits<IslandId>::max)());
            const auto island = static_cast<IslandId>(artifact.topology.islandCount++);
            ++result.stats.islands;
            queue.clear();
            queue.push_back(i);
            owners[i] = island;
            for (std::size_t head = 0; head < queue.size(); ++head) {
                checkpoint(input.canceled);
                for (auto neighbor : polygons[queue[head]].neighbors) {
                    if (owners[neighbor] == (std::numeric_limits<IslandId>::max)()) {
                        owners[neighbor] = island;
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        for (std::size_t i = 0; i < polygons.size(); ++i) {
            checkpoint(input.canceled);
            artifact.topology.polygons.push_back({polygons[i].ref, owners[i]});
        }

        std::set<SampleKey> unique;
        for (std::size_t i = 0; i < polygons.size(); ++i) {
            const auto& polygon = polygons[i];
            for (unsigned char edge = 0; edge < polygon.poly->vertCount; ++edge) {
                checkpoint(input.canceled);
                std::vector<std::pair<int, int>> covered;
                visitNeighbors(mesh, polygon, edge, index, input.canceled,
                    [&](std::size_t, int begin, int end) { covered.emplace_back(begin, end); });
                std::sort(covered.begin(), covered.end());
                const auto a = vertex(*polygon.tile, *polygon.poly, edge);
                const auto b = vertex(*polygon.tile, *polygon.poly,
                    static_cast<unsigned char>((edge + 1) % polygon.poly->vertCount));
                const auto emit = [&](int begin, int end) {
                    if (begin >= end) return;
                    BoundaryInterval interval{owners[i], polygon.ref, edge, begin / 255.0, end / 255.0,
                        interpolate(a, b, begin / 255.0), interpolate(a, b, end / 255.0)};
                    artifact.intervals.push_back(interval);
                    ++result.stats.boundaryIntervals;
                    sampleInterval(interval, config, input.canceled, artifact, unique, result.stats);
                };
                int cursor = 0;
                for (const auto& span : covered) {
                    checkpoint(input.canceled);
                    emit(cursor, span.first);
                    cursor = (std::max)(cursor, span.second);
                }
                emit(cursor, 255);
            }
        }
        checkpoint(input.canceled);
        result.value.emplace(std::move(artifact));
        result.status = StageStatus::Success;
    } catch (const Abort& error) { result.status = error.status; }
    catch (const std::bad_alloc&) { result.status = StageStatus::OutOfMemory; }
    catch (...) { result.status = StageStatus::CallbackFailed; }
    return result;
}

} // namespace detour_island_graph::v2
