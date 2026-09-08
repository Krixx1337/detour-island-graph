#include <doctest/doctest.h>
#include "../src/v2/BuildBudget.h"
#include <limits>

using namespace detour_island_graph::v2;

TEST_CASE("V2 allocation account charges simultaneous growth and survives scope") {
    auto account = std::make_shared<detail::AllocationAccount>();
    account->limits.maxAllocationBytes = 1024 * 1024;
    std::weak_ptr<detail::AllocationAccount> weak = account;
    BuildVector<int> escaped;
    {
        detail::AccountScope scope(account);
        BuildVector<int> values;
        values.reserve(32);
        const auto live = account->live.load();
        REQUIRE(live >= 32 * sizeof(int));
        // Reallocation must charge old + new storage before freeing old storage.
        account->limits.maxAllocationBytes = live + 64 * sizeof(int) - 1;
        bool rejected = false;
        try { values.reserve(64); }
        catch (const detail::BuildAbort& e) {
            rejected = true;
            CHECK(e.status == StageStatus::BudgetExceeded);
        }
        CHECK(rejected);
        CHECK(values.capacity() == 32);
        CHECK(account->live.load() == live);
        account->limits.maxAllocationBytes = 1024 * 1024;
        values.push_back(17);
        escaped = std::move(values);
    }
    account.reset();
    REQUIRE_FALSE(weak.expired());
    CHECK(escaped.front() == 17);
    // Copy outside production scope must not consume retained graph/build budget.
    const auto live = weak.lock()->live.load();
    auto copy = escaped;
    CHECK(weak.lock()->live.load() == live);
    CHECK(copy.front() == 17);
    escaped = BuildVector<int>{};
    CHECK(weak.expired());
}

TEST_CASE("V2 allocation byte multiplication overflow reports budget failure") {
    auto account = std::make_shared<detail::AllocationAccount>();
    account->limits.maxAllocationBytes = (std::numeric_limits<std::size_t>::max)();
    detail::AccountScope scope(account);
    BuildAllocator<std::uint64_t> allocator;
    bool rejected = false;
    try { auto* p = allocator.allocate((std::numeric_limits<std::size_t>::max)()); allocator.deallocate(p, 0); }
    catch (const detail::BuildAbort& e) { rejected = true; CHECK(e.status == StageStatus::BudgetExceeded); }
    REQUIRE(rejected);
    CHECK(account->diagnostics.exhausted == BudgetResource::AllocationBytes);
    CHECK(account->diagnostics.attempted == (std::numeric_limits<std::size_t>::max)());
    CHECK(account->live.load() == 0);
}
