#include "BuildBudget.h"
#include <algorithm>
#include <limits>

namespace detour_island_graph::v2::detail {
thread_local std::shared_ptr<AllocationAccount> activeAccount;
std::shared_ptr<AllocationAccount> currentAllocationAccount() noexcept { return activeAccount; }
namespace {
void enforce(AllocationAccount& account, BudgetResource resource, std::size_t current,
    std::size_t increment, std::size_t maximum) {
    const auto max = (std::numeric_limits<std::size_t>::max)();
    const bool overflow = increment > max - current;
    if (overflow || current > maximum || increment > maximum - current) {
        auto& d = account.diagnostics;
        if (d.exhausted == BudgetResource::None) {
            d.exhausted = resource;
            d.limit = maximum;
            d.attempted = overflow ? max : current + increment;
        }
        throw BuildAbort{StageStatus::BudgetExceeded};
    }
}
}
void chargeAllocation(const std::shared_ptr<AllocationAccount>& account, std::size_t bytes) {
    if (!account) return;
    const auto live = account->live.load(std::memory_order_relaxed);
    enforce(*account, BudgetResource::AllocationBytes, live, bytes, account->limits.maxAllocationBytes);
    account->live.fetch_add(bytes, std::memory_order_relaxed);
    account->diagnostics.peakAllocationBytes = (std::max)(account->diagnostics.peakAllocationBytes, live + bytes);
}
[[noreturn]] void allocationOverflow(const std::shared_ptr<AllocationAccount>& account) {
    if (!account) throw std::bad_array_new_length();
    auto& d = account->diagnostics;
    if (d.exhausted == BudgetResource::None) {
        d.exhausted = BudgetResource::AllocationBytes;
        d.limit = account->limits.maxAllocationBytes;
        d.attempted = (std::numeric_limits<std::size_t>::max)();
    }
    throw BuildAbort{StageStatus::BudgetExceeded};
}
void releaseAllocation(const std::shared_ptr<AllocationAccount>& account, std::size_t bytes) noexcept {
    if (account) account->live.fetch_sub(bytes, std::memory_order_relaxed);
}
void limit(BudgetResource resource, std::size_t current, std::size_t increment, std::size_t maximum) {
    if (activeAccount) enforce(*activeAccount, resource, current, increment, maximum);
}
void work() {
    if (!activeAccount) return;
    auto& d = activeAccount->diagnostics;
    limit(BudgetResource::WorkUnits, d.workUnits, 1, activeAccount->limits.maxWorkUnits);
    ++d.workUnits;
}
void checkCancel(const Cancel& canceled) {
    if (canceled && callback(canceled)) throw BuildAbort{StageStatus::Canceled};
}
} // namespace detour_island_graph::v2::detail
