#include <detour_island_graph/v2/Health.h>
#include <algorithm>
#include <cmath>
#include <tuple>

namespace detour_island_graph::v2 {
bool GraphHealth::healthy() const noexcept {
    return std::all_of(issues.begin(), issues.end(), [](auto n) { return n == 0; });
}
HealthResult analyzeGraphHealth(const CompiledGraph& g, const HealthOptions& o) {
    struct Canceled {};
    try {
        auto check = [&] { if (o.canceled && o.canceled()) throw Canceled{}; };
        check();
        const auto n = g.domain().size(), m = g.crossings().size();
        if (!o.maxIslands || !o.maxCrossings || !o.maxTraversals || !o.maxPolygons)
            return {StageStatus::InvalidInput, {}};
        if (n > o.maxIslands || m > o.maxCrossings || g.traversals().size() > o.maxTraversals ||
            g.polygonIslands().size() > o.maxPolygons || g.metrics().size() > o.maxIslands ||
            (!g.offsets().empty() && g.offsets().size() - 1 > o.maxIslands))
            return {StageStatus::BudgetExceeded, {}};
        GraphHealth h; h.islands.resize(n);
        auto issue = [&](HealthIssue kind, std::size_t i) {
            ++h.issues[static_cast<std::size_t>(kind)];
            if (h.examples.size() < o.maxIssueExamples) h.examples.push_back({kind, i});
        };
        auto finite = [](Point p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); };
        const auto& config = g.discovery();
        if (!std::isfinite(config.sampleSpacing) || config.sampleSpacing <= 0 ||
            !std::isfinite(g.identity().unitsPerMeter) || g.identity().unitsPerMeter <= 0 || !g.coverage().complete ||
            (g.policy() != CompilePolicy::GeometricOnly && g.policy() != CompilePolicy::ValidatedOnly) ||
            (g.policy() == CompilePolicy::ValidatedOnly && !g.validatorSupplied())) issue(HealthIssue::Metadata, 0);
        for (auto limit : {config.maxHorizontalGap, config.maxClimb, config.maxDrop})
            if (!std::isfinite(limit) || limit < 0) issue(HealthIssue::Metadata, 0);
        std::vector<std::size_t> polygons(n);
        // Sort ownership to make bounded examples independent of unordered iteration.
        std::vector<std::pair<dtPolyRef, IslandId>> ownership(g.polygonIslands().begin(), g.polygonIslands().end());
        std::sort(ownership.begin(), ownership.end());
        for (auto p : ownership) {
            check();
            if (!p.first || p.second >= n) issue(HealthIssue::Ownership, p.first);
            else ++polygons[p.second];
        }
        const bool metrics = g.metrics().size() == n;
        if (!g.metrics().empty() && !metrics) issue(HealthIssue::Metrics, 0);
        auto add = [&](HealthWeight& w, std::size_t i) {
            ++w.islands; w.polygons += polygons[i];
            if (metrics) w.surfaceArea = w.surfaceArea.value_or(0) + g.metrics()[i].surfaceArea;
        };
        if (metrics) h.included.surfaceArea = h.forward.surfaceArea = h.reverse.surfaceArea = h.mutual.surfaceArea = 0;
        for (std::size_t i = 0; i < n; ++i) {
            check();
            if (!polygons[i]) issue(HealthIssue::Ownership, i);
            if (metrics) {
                const auto& v = g.metrics()[i];
                if (v.polygonCount != polygons[i] || !std::isfinite(v.surfaceArea) || v.surfaceArea < 0 ||
                    !finite(v.boundsMin) || !finite(v.boundsMax) || v.boundsMin.x > v.boundsMax.x ||
                    v.boundsMin.y > v.boundsMax.y || v.boundsMin.z > v.boundsMax.z) issue(HealthIssue::Metrics, i);
            }
            if (g.includes(static_cast<IslandId>(i))) {
                add(h.included, i);
                if (!h.referenceIsland || polygons[i] > polygons[*h.referenceIsland]) h.referenceIsland = static_cast<IslandId>(i);
            } else if (g.domain()[i].state == DomainState::Excluded) ++h.excluded;
            else if (g.domain()[i].state == DomainState::Unexplored) ++h.unexplored;
            else issue(HealthIssue::Ownership, i);
        }
        std::vector<std::vector<std::size_t>> out(n), in(n);
        std::vector<std::array<std::size_t, 2>> seen(m);
        using Key = std::tuple<IslandId, dtPolyRef, float, float, float, IslandId, dtPolyRef, float, float, float>;
        std::vector<std::pair<Key, std::size_t>> keys;
        std::vector<int> outbound(n, -1);
        auto anchor = [&](const Anchor& a) {
            auto p = g.polygonIslands().find(a.polygon);
            return finite(a.position) && a.polygon && a.island < n && p != g.polygonIslands().end() && p->second == a.island;
        };
        for (std::size_t i = 0; i < m; ++i) {
            check(); const auto& c = g.crossings()[i]; const auto& a = c.crossing.a; const auto& b = c.crossing.b;
            bool valid = anchor(a) && anchor(b);
            if (!valid) issue(HealthIssue::Anchor, i);
            if (a.island >= b.island) issue(HealthIssue::Crossing, i);
            if (finite(a.position) && finite(b.position)) keys.push_back({Key(a.island,a.polygon,a.position.x,a.position.y,a.position.z,
                b.island,b.polygon,b.position.x,b.position.y,b.position.z), i});
            const unsigned count = unsigned(c.traversableAB) + unsigned(c.traversableBA);
            if (count == 2) ++h.bothDirections; else if (count) ++h.oneDirection; else ++h.neitherDirection;
            h.directedTraversals += count;
            for (unsigned r = 0; r < 2; ++r) {
                const auto& d = r ? c.crossing.ba : c.crossing.ab;
                const auto& fromAnchor = r ? b : a; const auto& toAnchor = r ? a : b;
                if (valid) {
                    const double dy = double(toAnchor.position.y) - fromAnchor.position.y;
                    const bool eligible = std::hypot(double(toAnchor.position.x) - fromAnchor.position.x,
                        double(toAnchor.position.z) - fromAnchor.position.z) <= config.maxHorizontalGap &&
                        dy <= config.maxClimb && -dy <= config.maxDrop;
                    auto& permission = outbound[fromAnchor.island];
                    if (d.geometricallyEligible != eligible || (!g.customOutboundPolicy() && !d.policyAllowed) ||
                        (permission != -1 && permission != int(d.policyAllowed))) issue(HealthIssue::Direction, i);
                    permission = int(d.policyAllowed);
                }
                if ((d.validation.state != ValidationState::Unknown && d.validation.state != ValidationState::Valid &&
                    d.validation.state != ValidationState::Invalid) ||
                    ((!g.validatorSupplied() || !d.geometricallyEligible || !d.policyAllowed) &&
                    (d.validation.state != ValidationState::Unknown || d.validation.reason != 0))) issue(HealthIssue::Direction, i);
                bool enabled = r ? c.traversableBA : c.traversableAB;
                const bool expected = d.geometricallyEligible && d.policyAllowed &&
                    (d.validation.state == ValidationState::Valid ||
                    (g.policy() == CompilePolicy::GeometricOnly && d.validation.state == ValidationState::Unknown));
                if (enabled != expected) issue(HealthIssue::Direction, i);
                if (!enabled || !valid) continue;
                auto from = r ? b.island : a.island, to = r ? a.island : b.island;
                if (!g.includes(from) || !g.includes(to) || from == to) { issue(HealthIssue::Direction, i); continue; }
                out[from].push_back(to); in[to].push_back(from);
            }
        }
        for (std::size_t i = 1; i < keys.size(); ++i) { check(); if (keys[i].first < keys[i-1].first) issue(HealthIssue::Crossing, keys[i].second); }
        std::sort(keys.begin(), keys.end());
        for (std::size_t i = 1; i < keys.size(); ++i) { check(); if (keys[i].first == keys[i-1].first) issue(HealthIssue::Duplicate, keys[i].second); }
        bool offsets = g.offsets().size() == n + 1 && g.offsets().front() == 0 && g.offsets().back() == g.traversals().size();
        if (offsets) for (std::size_t i = 0; i < n; ++i) if (g.offsets()[i] > g.offsets()[i+1]) offsets = false;
        if (!offsets) issue(HealthIssue::Adjacency, 0);
        else for (std::size_t i = 0; i < n; ++i) for (auto j = g.offsets()[i]; j < g.offsets()[i+1]; ++j) {
            check(); const auto& t = g.traversals()[j];
            if (t.crossing >= m) { issue(HealthIssue::Adjacency, j); continue; }
            const auto& c = g.crossings()[t.crossing];
            if (++seen[t.crossing][t.reverse] != 1 || (t.reverse ? c.crossing.b.island : c.crossing.a.island) != i ||
                !(t.reverse ? c.traversableBA : c.traversableAB)) issue(HealthIssue::Adjacency, j);
        }
        for (std::size_t i = 0; i < m; ++i) {
            check(); const auto& c = g.crossings()[i];
            if (seen[i][0] != unsigned(c.traversableAB) || seen[i][1] != unsigned(c.traversableBA)) issue(HealthIssue::Adjacency, i);
        }
        for (std::size_t i = 0; i < n; ++i) {
            check();
            for (auto* edges : {&out[i], &in[i]}) { std::sort(edges->begin(), edges->end()); edges->erase(std::unique(edges->begin(),edges->end()),edges->end()); }
            h.islands[i].outgoingNeighbors = out[i].size(); h.islands[i].incomingNeighbors = in[i].size();
            if (g.includes(static_cast<IslandId>(i))) {
                h.zeroOutgoing += out[i].empty(); h.zeroIncoming += in[i].empty(); h.isolated += out[i].empty() && in[i].empty();
            }
        }
        // Iterative Kosaraju avoids stack overflow on long directed chains.
        std::vector<bool> visited(n); std::vector<std::size_t> order;
        std::vector<std::pair<std::size_t,std::size_t>> dfs;
        for (std::size_t i = 0; i < n; ++i) if (g.includes(static_cast<IslandId>(i)) && !visited[i]) {
            visited[i] = true; dfs.emplace_back(i,0);
            while (!dfs.empty()) {
                check(); auto& frame = dfs.back();
                if (frame.second == out[frame.first].size()) { order.push_back(frame.first); dfs.pop_back(); }
                else { auto next = out[frame.first][frame.second++]; if (!visited[next]) { visited[next] = true; dfs.emplace_back(next,0); } }
            }
        }
        std::vector<std::size_t> stack;
        for (auto it = order.rbegin(); it != order.rend(); ++it) if (!h.islands[*it].component) {
            const auto id = h.components.size(); h.components.emplace_back();
            h.islands[*it].component = id; stack.push_back(*it);
            while (!stack.empty()) {
                check(); auto v = stack.back(); stack.pop_back(); add(h.components.back(),v);
                for (auto w : in[v]) if (!h.islands[w].component) { h.islands[w].component = id; stack.push_back(w); }
            }
        }
        if (h.referenceIsland) for (bool reverse : {false,true}) {
            std::fill(visited.begin(), visited.end(), false); stack.push_back(*h.referenceIsland); visited[*h.referenceIsland] = true;
            while (!stack.empty()) {
                check(); auto v = stack.back(); stack.pop_back();
                if (reverse) { h.islands[v].reverse = true; add(h.reverse,v); }
                else { h.islands[v].forward = true; add(h.forward,v); }
                for (auto w : (reverse ? in[v] : out[v])) if (!visited[w]) { visited[w] = true; stack.push_back(w); }
            }
        }
        for (std::size_t i = 0; i < n; ++i) { check(); if (h.islands[i].forward && h.islands[i].reverse) add(h.mutual,i); }
        return {StageStatus::Success, std::move(h)};
    } catch (const Canceled&) { return {StageStatus::Canceled,{}}; }
      catch (const std::bad_alloc&) { return {StageStatus::OutOfMemory,{}}; }
      catch (...) { return {StageStatus::CallbackFailed,{}}; }
}
}
