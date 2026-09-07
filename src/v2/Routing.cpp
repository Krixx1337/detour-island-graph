#include <detour_island_graph/v2/Routing.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <utility>
#include <vector>

namespace detour_island_graph::v2 {
namespace {

constexpr std::size_t kNoPortal = (std::numeric_limits<std::size_t>::max)();

struct Abort { RouteStatus status; };

bool finite(Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

void checkpoint(const Cancel& canceled) {
    if (canceled && canceled()) throw Abort{RouteStatus::Canceled};
}

float euclidean(Point a, Point b) {
    const double dx = double(b.x) - a.x;
    const double dy = double(b.y) - a.y;
    const double dz = double(b.z) - a.z;
    return float(std::sqrt(dx * dx + dy * dy + dz * dz));
}

bool usable(float value) {
    return std::isfinite(value) && value >= 0;
}

} // namespace

RouteResult findRoute(const CompiledGraph& graph, IslandId startIsland, IslandId endIsland,
    Point startPosition, Point endPosition, const RouteOptions& options, RouteScratch* scratch) {
    RouteResult result;
    try {
        checkpoint(options.canceled);
        result.stats.estimatedTransferCost = !options.transferCost || options.transferCostEstimated;
        result.stats.estimatedCrossingCost = !options.crossingCost || options.crossingCostEstimated;
        result.stats.estimatedCost = result.stats.estimatedTransferCost || result.stats.estimatedCrossingCost;
        const std::size_t islandCount =
            graph.offsets().empty() ? 0 : graph.offsets().size() - 1;
        if (!finite(startPosition) || !finite(endPosition)) {
            result.status = RouteStatus::InvalidInput;
            return result;
        }
        if (startIsland >= islandCount || endIsland >= islandCount) {
            result.status = RouteStatus::InvalidIsland;
            return result;
        }
        if (!graph.includes(startIsland) || !graph.includes(endIsland)) {
            result.status = RouteStatus::OutOfDomain;
            return result;
        }
        if (startIsland == endIsland) {
            result.status = RouteStatus::SameIsland;
            return result;
        }
        const RouteCostContext context{graph, startIsland, endIsland};
        const bool useAStar = !options.transferCost && !options.crossingCost;
        result.stats.usedAStar = useAStar;

        RouteScratch local;
        RouteScratch* work = scratch ? scratch : &local;
        work->states.assign(graph.traversals().size(), RouteScratch::State{});
        work->heap.clear();

        const auto transfer = [&](IslandId island, const Anchor& from, const Anchor& to,
                                  float& cost) {
            cost = options.transferCost ? options.transferCost(island, from, to)
                                        : euclidean(from.position, to.position);
            checkpoint(options.canceled);
            return usable(cost);
        };
        const auto gap = [&](const CompiledCrossing& crossing, bool reverse, float& cost) {
            cost = options.crossingCost ? options.crossingCost(crossing, reverse, context)
                                        : euclidean(crossing.crossing.a.position,
                                              crossing.crossing.b.position);
            checkpoint(options.canceled);
            return usable(cost);
        };
        const auto heuristic = [&](const Anchor& from) {
            return useAStar ? euclidean(from.position, endPosition) : 0.0f;
        };
        auto push = [&](std::size_t portal, RouteLink leg, float gCost, float hCost) {
            if (options.maxQueuedPortals > 0 &&
                result.stats.queuedPortals + 1 > options.maxQueuedPortals)
                throw Abort{RouteStatus::BudgetExceeded};
            auto& state = work->states[portal];
            state.cost = gCost;
            state.closed = false;
            state.leg = std::move(leg);
            work->heap.push_back({gCost + hCost, gCost, portal});
            std::push_heap(work->heap.begin(), work->heap.end(),
                [](const RouteScratch::HeapEntry& a, const RouteScratch::HeapEntry& b) {
                    return a.bound > b.bound;
                });
            ++result.stats.queuedPortals;
            result.stats.peakOpenSetSize =
                (std::max)(result.stats.peakOpenSetSize, work->heap.size());
        };

        const Anchor startAnchor{startIsland, 0, startPosition};
        const auto& offsets = graph.offsets();
        for (std::size_t portal = offsets[startIsland]; portal < offsets[startIsland + 1];
             ++portal) {
            checkpoint(options.canceled);
            const Traversal& traversal = graph.traversals()[portal];
            if (traversal.crossing >= graph.crossings().size()) {
                result.status = RouteStatus::InvalidInput;
                return result;
            }
            const CompiledCrossing& compiled = graph.crossings()[traversal.crossing];
            const Anchor from = traversal.reverse ? compiled.crossing.b : compiled.crossing.a;
            const Anchor to = traversal.reverse ? compiled.crossing.a : compiled.crossing.b;
            if (from.island != startIsland) {
                result.status = RouteStatus::InvalidInput;
                return result;
            }
            if (options.crossingFilter && !options.crossingFilter(compiled, traversal.reverse, context))
                continue;
            float transferCost = 0, gapCost = 0;
            if (!transfer(startIsland, startAnchor, from, transferCost)) continue;
            if (!gap(compiled, traversal.reverse, gapCost)) continue;
            const float gCost = transferCost + gapCost;
            if (!usable(gCost)) continue;
            push(portal, {traversal.crossing, traversal.reverse, from, to}, gCost, heuristic(to));
        }

        std::size_t bestPortal = kNoPortal;
        float bestCost = std::numeric_limits<float>::infinity();
        const Anchor endAnchor{endIsland, 0, endPosition};
        while (!work->heap.empty()) {
            checkpoint(options.canceled);
            std::pop_heap(work->heap.begin(), work->heap.end(),
                [](const RouteScratch::HeapEntry& a, const RouteScratch::HeapEntry& b) {
                    return a.bound > b.bound;
                });
            const auto entry = work->heap.back();
            work->heap.pop_back();
            if (entry.portal >= work->states.size()) {
                result.status = RouteStatus::InvalidInput;
                return result;
            }
            auto& state = work->states[entry.portal];
            if (state.closed || entry.cost != state.cost) continue;
            // Stale entries consume no expansion budget. A proven result at
            // exactly the cap succeeds without expanding irrelevant portals.
            if (bestPortal != kNoPortal && entry.bound >= bestCost) break;
            if (options.maxExpandedPortals > 0 &&
                result.stats.expandedPortals >= options.maxExpandedPortals)
                throw Abort{RouteStatus::BudgetExceeded};
            state.closed = true;
            ++result.stats.expandedPortals;

            const Anchor& arrivedAt = state.leg.to;
            if (arrivedAt.island == endIsland) {
                float finish = 0;
                if (transfer(endIsland, arrivedAt, endAnchor, finish) && usable(state.cost + finish) &&
                    state.cost + finish < bestCost) {
                    bestCost = state.cost + finish;
                    bestPortal = entry.portal;
                }
            }
            if (entry.bound >= bestCost) continue;

            const IslandId island = arrivedAt.island;
            if (island >= islandCount) {
                result.status = RouteStatus::InvalidInput;
                return result;
            }
            for (std::size_t next = offsets[island]; next < offsets[island + 1]; ++next) {
                checkpoint(options.canceled);
                const Traversal& traversal = graph.traversals()[next];
                if (traversal.crossing >= graph.crossings().size()) {
                    result.status = RouteStatus::InvalidInput;
                    return result;
                }
                const CompiledCrossing& compiled = graph.crossings()[traversal.crossing];
                const Anchor from =
                    traversal.reverse ? compiled.crossing.b : compiled.crossing.a;
                const Anchor to = traversal.reverse ? compiled.crossing.a : compiled.crossing.b;
                if (from.island != island) {
                    result.status = RouteStatus::InvalidInput;
                    return result;
                }
                if (options.crossingFilter &&
                    !options.crossingFilter(compiled, traversal.reverse, context))
                    continue;
                float transferCost = 0, gapCost = 0;
                if (!transfer(island, arrivedAt, from, transferCost)) continue;
                if (!gap(compiled, traversal.reverse, gapCost)) continue;
                const float gCost = state.cost + transferCost + gapCost;
                if (!usable(gCost)) continue;
                auto& target = work->states[next];
                if (gCost >= target.cost) continue;
                const float bound = gCost + heuristic(to);
                if (bestPortal != kNoPortal && bound >= bestCost) continue;
                target.cost = gCost;
                target.previous = entry.portal;
                push(next, {traversal.crossing, traversal.reverse, from, to}, gCost,
                    heuristic(to));
            }
        }

        checkpoint(options.canceled);
        if (bestPortal == kNoPortal) {
            result.status = RouteStatus::NoPath;
            return result;
        }
        Route route;
        for (std::size_t cursor = bestPortal; cursor != kNoPortal;
             cursor = work->states[cursor].previous)
            route.legs.push_back(work->states[cursor].leg);
        std::reverse(route.legs.begin(), route.legs.end());
        route.totalCost = bestCost;
        result.value.emplace(std::move(route));
        result.status = RouteStatus::Success;
    } catch (const Abort& error) {
        result.status = error.status;
    } catch (const std::bad_alloc&) {
        result.status = RouteStatus::OutOfMemory;
    } catch (...) {
        result.status = RouteStatus::CallbackFailed;
    }
    return result;
}

} // namespace detour_island_graph::v2
