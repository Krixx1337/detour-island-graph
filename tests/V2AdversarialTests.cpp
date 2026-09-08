#include <doctest/doctest.h>
#include "V2Adversarial.h"
using namespace adversarial;

TEST_CASE("V2 adversarial discovery agrees with analytic rectangles under stress") {
    CHECK_NOTHROW(runScenarios());
}
TEST_CASE("V2 narrow landing window documents finite sampling coverage") {
    for(float spacing:{4.f,1.f}) {
        auto result=landingWindow(spacing);
        REQUIRE(result.status==StageStatus::Success);REQUIRE(result.graph);
        CHECK(result.graph->traversals().empty()==(spacing==4));
    }
}
TEST_CASE("V2 stress failure and cancellation leave later builds usable") {
    std::vector<Rect> rects;for(unsigned i=0;i<64;++i)rects.push_back({0,0,2,2,float(i)});
    auto mesh=makeMesh(rects);BuildInput input;input.navMesh=mesh.get();
    auto config=DiscoveryConfig{2,1,64,64,10000,100000};
    for(bool allocation:{false,true}) {
        auto options=production();
        if(allocation)options.limits.maxAllocationBytes=1024;else options.limits.maxWorkUnits=100;
        auto failed=buildGraphBounded(input,config,options);
        CHECK(failed.status==StageStatus::BudgetExceeded);CHECK_FALSE(failed.graph);
        CHECK(failed.budget.exhausted==(allocation?BudgetResource::AllocationBytes:BudgetResource::WorkUnits));
    }
    int calls=0;input.canceled=[&]{return ++calls==100;};
    auto canceled=buildGraphBounded(input,config,production());CHECK(canceled.status==StageStatus::Canceled);CHECK_FALSE(canceled.graph);
    input.canceled={};CHECK(buildGraphBounded(input,config,production()).status==StageStatus::Success);
}
