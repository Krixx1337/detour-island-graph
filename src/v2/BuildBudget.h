#pragma once
#include <detour_island_graph/v2/Build.h>
#include <atomic>
#include <map>
#include <set>
#include <utility>

namespace detour_island_graph::v2::detail {
struct BuildAbort { StageStatus status; };
struct AllocationAccount {
    BuildLimits limits;
    BudgetDiagnostics diagnostics;
    std::atomic<std::size_t> live{0};
};
extern thread_local std::shared_ptr<AllocationAccount> activeAccount;
class AccountScope {
public:
    explicit AccountScope(std::shared_ptr<AllocationAccount> account) noexcept
        : previous_(std::move(activeAccount)) { activeAccount = std::move(account); }
    ~AccountScope() { activeAccount = std::move(previous_); }
    AccountScope(const AccountScope&) = delete;
    AccountScope& operator=(const AccountScope&) = delete;
private:
    std::shared_ptr<AllocationAccount> previous_;
};
// User callbacks may reenter the library. Never inherit a caller's build budget.
template<class F, class... A> decltype(auto) callback(const F& f, A&&... args) {
    AccountScope suspended(nullptr);
    try { return f(std::forward<A>(args)...); }
    catch (...) { throw BuildAbort{StageStatus::CallbackFailed}; }
}
void limit(BudgetResource resource, std::size_t current, std::size_t increment, std::size_t maximum);
void work();
void checkCancel(const Cancel& canceled);
template<class T> using Set = std::set<T, std::less<T>, BuildAllocator<T>>;
template<class K, class V> using Map = std::map<K, V, std::less<K>, BuildAllocator<std::pair<const K, V>>>;
} // namespace detour_island_graph::v2::detail
