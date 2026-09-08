#include <doctest/doctest.h>
#include <detour_island_graph/v2/Build.h>
#include <detour_island_graph/v2/Serialization.h>
#include <detour_island_graph/v2/Health.h>
#include "V2RouteOracle.h"
#include <DetourAlloc.h>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
using namespace detour_island_graph::v2;
using Bytes = std::vector<unsigned char>;
struct DeleteMesh { void operator()(dtNavMesh* p) const { dtFreeNavMesh(p); } };
using Mesh = std::unique_ptr<dtNavMesh, DeleteMesh>;
struct Header { int magic, version, tiles; dtNavMeshParams params; };
struct TileHeader { dtTileRef ref; int size; };
template<class T> T take(const Bytes& bytes, std::size_t& pos) {
    if (pos > bytes.size() || sizeof(T) > bytes.size() - pos) throw std::runtime_error("Truncated MSET");
    T value; std::memcpy(&value, bytes.data() + pos, sizeof(T)); pos += sizeof(T); return value;
}
Bytes fixture(const char* name) {
    std::ifstream in(std::string(DIG_FIXTURE_DIR) + "/" + name + "/" + name + ".nav", std::ios::binary);
    if (!in) throw std::runtime_error("Missing fixture");
    return Bytes(std::istreambuf_iterator<char>(in), {});
}
// Test-only reader for the extractor's native MSET ABI and trusted Detour payloads.
Mesh load(const Bytes& bytes) {
    std::size_t pos = 0;
    const auto h = take<Header>(bytes, pos);
    if (h.magic != 0x4d534554 || h.version != 1 || h.tiles <= 0 || h.tiles > 1024 ||
        h.params.maxTiles < h.tiles || h.params.maxTiles > 1024 || h.params.maxPolys <= 0 ||
        h.params.maxPolys > (1 << 22) || !(h.params.tileWidth > 0) || !(h.params.tileHeight > 0))
        throw std::runtime_error("Invalid MSET header");
    Mesh mesh(dtAllocNavMesh());
    if (!mesh || dtStatusFailed(mesh->init(&h.params))) throw std::runtime_error("Mesh init failed");
    for (int i = 0; i < h.tiles; ++i) {
        const auto tile = take<TileHeader>(bytes, pos);
        if (tile.size < int(sizeof(dtMeshHeader)) || std::size_t(tile.size) > bytes.size() - pos)
            throw std::runtime_error("Invalid tile size");
        std::size_t headerPos = pos;
        const auto native = take<dtMeshHeader>(bytes, headerPos);
        if (native.magic != DT_NAVMESH_MAGIC || native.version != DT_NAVMESH_VERSION)
            throw std::runtime_error("Incompatible Detour tile");
        std::size_t required = (sizeof(dtMeshHeader) + 3) & ~std::size_t(3);
        const auto section = [&](int count, std::size_t stride) {
            if (count < 0 || std::size_t(count) > std::size_t(tile.size) / stride)
                throw std::runtime_error("Invalid tile section");
            required += (std::size_t(count) * stride + 3) & ~std::size_t(3);
        };
        section(native.vertCount, sizeof(float) * 3);
        section(native.polyCount, sizeof(dtPoly));
        section(native.maxLinkCount, sizeof(dtLink));
        section(native.detailMeshCount, sizeof(dtPolyDetail));
        section(native.detailVertCount, sizeof(float) * 3);
        section(native.detailTriCount, 4);
        section(native.bvNodeCount, sizeof(dtBVNode));
        section(native.offMeshConCount, sizeof(dtOffMeshConnection));
        if (required != std::size_t(tile.size)) throw std::runtime_error("Tile layout size mismatch");
        auto* data = static_cast<unsigned char*>(dtAlloc(tile.size, DT_ALLOC_PERM));
        if (!data) throw std::bad_alloc();
        std::memcpy(data, bytes.data() + pos, tile.size); pos += tile.size;
        if (dtStatusFailed(mesh->addTile(data, tile.size, DT_TILE_FREE_DATA, tile.ref, nullptr))) {
            dtFree(data); throw std::runtime_error("Tile load failed");
        }
    }
    if (pos != bytes.size()) throw std::runtime_error("Trailing MSET data");
    return mesh;
}
std::string serialize(const CompiledGraph& g) {
    std::ostringstream out(std::ios::binary);
    REQUIRE(GraphSerializer::write(out, g) == SerializationStatus::Success);
    return out.str();
}
ProductionBuildOptions options(std::size_t batch = 64) {
    ProductionBuildOptions o;
    o.limits = {1024 * 1024, 100000, 1000, 2000, 1000, 2000, 2000, 4000};
    o.sampleBatchSize = o.candidateBatchSize = batch;
    return o;
}
}

TEST_CASE("V2 real fixtures match reference discovery across batches") {
    for (const auto* name : {"pandora", "steelribs"}) {
        INFO(std::string(name));
        auto mesh = load(fixture(name));
        int tiles = 0;
        for (int i = 0; i < mesh->getMaxTiles(); ++i) {
            const auto* tile = static_cast<const dtNavMesh&>(*mesh).getTile(i);
            if (tile && tile->header) ++tiles;
        }
        CHECK(tiles == 2);
        BuildInput input; input.navMesh = mesh.get();
        DiscoveryConfig config{0.5f, 2, 1, 3, 10000, 10000};
        const auto reference = buildGraph(input, config);
        REQUIRE(reference.status == StageStatus::Success);
        REQUIRE(reference.compilation.value);
        const auto& graph = **reference.compilation.value;
        const auto health = analyzeGraphHealth(graph);
        REQUIRE(health.status == StageStatus::Success);
        REQUIRE(health.value);
        CHECK(health.value->healthy());
        CHECK_NOTHROW(fixture_oracle::checkAllRoutes(graph, *mesh));
        REQUIRE_FALSE(graph.crossings().empty());
        const bool pandora = std::string(name) == "pandora";
        CHECK(reference.sampling.value->topology.polygons.size() == (pandora ? 134 : 69));
        CHECK(reference.sampling.value->topology.islandCount == (pandora ? 25 : 15));
        CHECK(reference.sampling.stats.boundaryIntervals == (pandora ? 300 : 145));
        CHECK(graph.crossings().size() == (pandora ? 99 : 236));
        for (const auto& crossing : reference.validation.value->crossings) {
            CHECK(crossing.ab.validation.state == ValidationState::Unknown);
            CHECK(crossing.ba.validation.state == ValidationState::Unknown);
        }
        for (const auto& p : reference.sampling.value->topology.polygons)
            CHECK(mesh->isValidPolyRef(p.polygon));
        for (std::size_t batch : {1u, 64u}) {
            const auto built = buildGraphBounded(input, config, options(batch));
            REQUIRE(built.status == StageStatus::Success);
            REQUIRE(built.graph);
            CHECK(serialize(*built.graph) == serialize(graph));
            CHECK(built.budget.peakSampleBatch <= batch);
            CHECK(built.budget.peakCandidateBatch <= batch);
            std::cout << name << " batch=" << batch << " polygons=" << built.stats.eligiblePolygons
                << " islands=" << reference.sampling.value->topology.islandCount
                << " intervals=" << built.stats.boundaryIntervals << " unique=" << built.budget.uniqueCrossings
                << " bytes=" << built.budget.peakAllocationBytes << " work=" << built.budget.workUnits
                << " ms=" << built.timings.totalMs << '\n';
        }
        CHECK(reference.validation.stats.validDirections == 0);
        CHECK(reference.validation.stats.validatorCalls == 0);
        ValidationOptions unknown;
        unknown.validator = [](const ValidationRequest&) { return ValidationResult{}; };
        auto strict = buildGraphBounded(input, config, options(), unknown, {CompilePolicy::ValidatedOnly, {}});
        REQUIRE(strict.status == StageStatus::Success);
        CHECK(strict.stats.compiledDirections == 0);
        for (bool memory : {false, true}) {
            auto o = options();
            if (memory) o.limits.maxAllocationBytes = 1; else o.limits.maxWorkUnits = 1;
            const auto failed = buildGraphBounded(input, config, o);
            CHECK(failed.status == StageStatus::BudgetExceeded);
            CHECK(failed.budget.exhausted == (memory ? BudgetResource::AllocationBytes : BudgetResource::WorkUnits));
            CHECK_FALSE(failed.graph);
        }
        input.canceled = [] { return true; };
        const auto canceled = buildGraphBounded(input, config, options());
        CHECK(canceled.status == StageStatus::Canceled);
        CHECK_FALSE(canceled.graph);
    }
}

TEST_CASE("Real fixture loader rejects malformed envelopes") {
    const auto original = fixture("pandora");
    for (std::size_t length : {std::size_t(0), sizeof(Header) - 1, sizeof(Header) + sizeof(TileHeader) - 1, original.size() - 1}) {
        Bytes truncated(original.begin(), original.begin() + length);
        CHECK_THROWS_AS(load(truncated), std::runtime_error);
    }
    for (std::size_t offset : {offsetof(Header, magic), offsetof(Header, version), offsetof(Header, tiles), sizeof(Header) + offsetof(TileHeader, size)}) {
        auto broken = original;
        const int invalid = -1;
        std::memcpy(broken.data() + offset, &invalid, sizeof(invalid));
        CHECK_THROWS_AS(load(broken), std::runtime_error);
    }
    for (std::size_t offset : {offsetof(dtMeshHeader, version), offsetof(dtMeshHeader, vertCount)}) {
        auto broken = original;
        const int invalid = -1;
        std::memcpy(broken.data() + sizeof(Header) + sizeof(TileHeader) + offset, &invalid, sizeof(invalid));
        CHECK_THROWS_AS(load(broken), std::runtime_error);
    }
}
