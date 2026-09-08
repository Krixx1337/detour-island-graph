#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <new>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace detour_island_graph::v2 {
namespace detail {
struct AllocationAccount;
std::shared_ptr<AllocationAccount> currentAllocationAccount() noexcept;
void chargeAllocation(const std::shared_ptr<AllocationAccount>&, std::size_t);
[[noreturn]] void allocationOverflow(const std::shared_ptr<AllocationAccount>&);
void releaseAllocation(const std::shared_ptr<AllocationAccount>&, std::size_t) noexcept;
}

// Owned storage keeps its account alive after the build returns. Copies made
// outside a production call use an unbounded account; moving preserves ownership.
template<class T> class BuildAllocator {
public:
    using value_type = T;
    using propagate_on_container_move_assignment = std::true_type;
    using propagate_on_container_swap = std::true_type;
    // MSVC debug containers allocate iterator proxies in their default
    // constructors. Keep those constructors throwable when the budget fails.
    BuildAllocator() : account_(detail::currentAllocationAccount()) {}
    template<class U> BuildAllocator(const BuildAllocator<U>& other) noexcept : account_(other.account_) {}
    BuildAllocator select_on_container_copy_construction() const noexcept { return {}; }
    T* allocate(std::size_t n) {
        if (n > (std::numeric_limits<std::size_t>::max)() / sizeof(T)) detail::allocationOverflow(account_);
        const auto bytes = n * sizeof(T);
        detail::chargeAllocation(account_, bytes);
        try { return std::allocator<T>{}.allocate(n); }
        catch (...) { detail::releaseAllocation(account_, bytes); throw; }
    }
    void deallocate(T* p, std::size_t n) noexcept {
        std::allocator<T>{}.deallocate(p, n);
        detail::releaseAllocation(account_, n * sizeof(T));
    }
    template<class U> bool operator==(const BuildAllocator<U>& rhs) const noexcept { return account_ == rhs.account_; }
    template<class U> bool operator!=(const BuildAllocator<U>& rhs) const noexcept { return !(*this == rhs); }
private:
    template<class> friend class BuildAllocator;
    std::shared_ptr<detail::AllocationAccount> account_;
};

template<class T> using BuildVector = std::vector<T, BuildAllocator<T>>;
template<class K, class V> using BuildUnorderedMap = std::unordered_map<K, V,
    std::hash<K>, std::equal_to<K>, BuildAllocator<std::pair<const K, V>>>;
} // namespace detour_island_graph::v2
