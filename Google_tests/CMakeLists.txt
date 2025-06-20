project(Google_tests)
set(GOOGLETEST_VERSION 1.14.0)
add_subdirectory(lib)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(Google_Tests_run EmptyTest.cpp)
target_link_libraries(Google_Tests_run QuadcopterPropellerDetector_lib)
target_link_libraries(Google_Tests_run gtest gtest_main)