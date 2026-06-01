cmake_minimum_required(VERSION 3.15)

project(DetourIslandGraph VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(DETOUR_ISLAND_GRAPH_BUILD_TESTS_DEFAULT ON)
else()
    set(DETOUR_ISLAND_GRAPH_BUILD_TESTS_DEFAULT OFF)
endif()
option(DETOUR_ISLAND_GRAPH_BUILD_TESTS
    "Build DetourIslandGraph tests"
    ${DETOUR_ISLAND_GRAPH_BUILD_TESTS_DEFAULT})
option(DETOUR_ISLAND_GRAPH_FETCH_DETOUR
    "Fetch pinned upstream RecastNavigation when Detour is not provided" ON)
option(DETOUR_ISLAND_GRAPH_DT_POLYREF64
    "Build fetched Detour with 64-bit polygon references" OFF)
set(DETOUR_ISLAND_GRAPH_DETOUR_INCLUDE_DIR "" CACHE PATH
    "Directory containing DetourNavMesh.h when Detour is not provided as a CMake target")
set(DETOUR_ISLAND_GRAPH_DETOUR_LIBRARY "" CACHE FILEPATH
    "Optional prebuilt Detour library used with DETOUR_ISLAND_GRAPH_DETOUR_INCLUDE_DIR")

add_library(detour_island_graph
    src/IslandGraph.cpp
    src/IslandGraphBoundaries.cpp
    src/IslandGraphBuilder.cpp
    src/IslandGraphDiscovery.cpp
    src/IslandGraphPruning.cpp
    src/IslandGraphScanner.cpp
    src/IslandGraphPathfinder.cpp
    src/IslandGraphSerializer.cpp
    src/IslandGraphTopology.cpp
)
add_library(detour_island_graph::detour_island_graph ALIAS detour_island_graph)

target_compile_features(detour_island_graph PUBLIC cxx_std_17)
target_include_directories(detour_island_graph
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(NOT TARGET Detour AND
   NOT TARGET RecastNavigation::Detour AND
   NOT DETOUR_ISLAND_GRAPH_DETOUR_INCLUDE_DIR AND
   DETOUR_ISLAND_GRAPH_FETCH_DETOUR)
    include(FetchContent)
    FetchContent_Declare(
        recastnavigation
        GIT_REPOSITORY https://github.com/recastnavigation/recastnavigation.git
        GIT_TAG v1.6.0
        GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(recastnavigation)
    if(NOT recastnavigation_POPULATED)
        FetchContent_Populate(recastnavigation)
    endif()
    set(SOVERSION 1)
    set(LIB_VERSION 1.6.0)
    set(RECASTNAVIGATION_DT_POLYREF64 ${DETOUR_ISLAND_GRAPH_DT_POLYREF64})
    set(RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER OFF)
    set(RECASTNAVIGATION_ENABLE_ASSERTS "$<CONFIG:Debug>")
    add_subdirectory(
        "${recastnavigation_SOURCE_DIR}/Detour"
        "${recastnavigation_BINARY_DIR}/Detour"
        EXCLUDE_FROM_ALL
    )
endif()

if(TARGET Detour)
    target_link_libraries(detour_island_graph PUBLIC "$<BUILD_INTERFACE:Detour>")
elseif(TARGET RecastNavigation::Detour)
    target_link_libraries(detour_island_graph PUBLIC "$<BUILD_INTERFACE:RecastNavigation::Detour>")
elseif(DETOUR_ISLAND_GRAPH_DETOUR_INCLUDE_DIR)
    target_include_directories(detour_island_graph PUBLIC
        "$<BUILD_INTERFACE:${DETOUR_ISLAND_GRAPH_DETOUR_INCLUDE_DIR}>")
    if(DETOUR_ISLAND_GRAPH_DETOUR_LIBRARY)
        target_link_libraries(detour_island_graph PUBLIC
            "$<BUILD_INTERFACE:${DETOUR_ISLAND_GRAPH_DETOUR_LIBRARY}>")
    endif()
else()
    message(FATAL_ERROR
        "Provide a Detour CMake target (Detour or RecastNavigation::Detour), set "
        "DETOUR_ISLAND_GRAPH_DETOUR_INCLUDE_DIR to the directory containing DetourNavMesh.h, "
        "or enable DETOUR_ISLAND_GRAPH_FETCH_DETOUR.")
endif()

include(CMakePackageConfigHelpers)

install(TARGETS detour_island_graph
    EXPORT detour_island_graph_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT detour_island_graph_targets
    FILE detour_island_graphTargets.cmake
    NAMESPACE detour_island_graph::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/detour_island_graph
)
configure_package_config_file(
    cmake/detour_island_graphConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/detour_island_graphConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/detour_island_graph
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/detour_island_graphConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/detour_island_graphConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/detour_island_graphConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/detour_island_graph
)

if(DETOUR_ISLAND_GRAPH_BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    set(DOCTEST_NO_INSTALL ON CACHE BOOL "Do not install doctest with DetourIslandGraph" FORCE)
    FetchContent_Declare(
        doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG v2.5.2
    )
    FetchContent_MakeAvailable(doctest)

    add_executable(detour_island_graph_tests
        tests/IslandGraphTests.cpp
    )
    target_link_libraries(detour_island_graph_tests
        PRIVATE
            detour_island_graph
            doctest::doctest
    )

    # Keep Release builds clean: only auto-build tests in Debug.
    set_target_properties(detour_island_graph_tests PROPERTIES
        EXCLUDE_FROM_DEFAULT_BUILD_RELEASE TRUE
        EXCLUDE_FROM_DEFAULT_BUILD_MINSIZEREL TRUE
        EXCLUDE_FROM_DEFAULT_BUILD_RELWITHDEBINFO TRUE
    )

    include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
    doctest_discover_tests(detour_island_graph_tests)
endif()
