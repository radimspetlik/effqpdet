cmake_minimum_required(VERSION 3.26)

project(detector)

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    message("Windows system detected")
    include_directories("c:/Program Files/Prophesee/third_party/include")
    link_directories("c:/Program Files/Prophesee/third_party/lib" "c:/Program Files/Prophesee/lib")

    set(BOOST_ROOT  "c:/Program Files/Prophesee/third_party")
    set(Boost_INCLUDE_DIR "c:/Program Files/Prophesee/third_party/include")
    set(Boost_LIBRARY_DIR "c:/Program Files/Prophesee/third_party/lib")
    set(Boost_COMPILER "-vc140")
    set(Boost_ARCHITECTURE "-mt")

    set(TIFF_INCLUDE_DIR "c:/Program Files/Prophesee/third_party/include")
    set(TIFF_LIBRARY "c:/Program Files/Prophesee/third_party/lib")

    set(ZLIB_LIBRARY "c:/Program Files/Prophesee/third_party/lib")
    set(ZLIB_INCLUDE_DIR "c:/Program Files/Prophesee/third_party/include")

    set(Protobuf_INCLUDE_DIR "c:/Program Files/Prophesee/third_party/include")
    set(Protobuf_LIBRARIES "c:/Program Files/Prophesee/third_party/lib/libprotobuf.lib" "c:/Program Files/Prophesee/third_party/lib/libprotoc.lib")

    #set(YAML_CPP_INCLUDE_DIR "c:/Users/radio/lib/yaml-cpp-0.8.0/include")
    #set(YAML_CPP_LIBRARY_DIR "c:/Users/radio/lib/yaml-cpp-0.8.0/lib")

    set(HDF5_INCLUDE_DIRS "c:/Users/radio/.vcpkg-clion/vcpkg/installed/x64-windows/include")

    find_package(HDF5 CONFIG REQUIRED COMPONENTS C CXX)
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message("Linux system detected")
    add_definitions(-w -Wno-dev)

    include_directories("/usr/include/hdf5/serial")
    find_package(HDF5 REQUIRED COMPONENTS C CXX)

    include_directories("/usr/include/metavision")
    link_directories("/usr/lib/metavision")
endif()

find_package(yaml-cpp REQUIRED)
find_package(MetavisionSDK COMPONENTS base core driver ui REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

set(EXECUTABLE qpdet)
set(CORRELATION_RESPONSE_MEASUREMENT_EXECUTABLE correlation_response_measurement)
set(PEAK_DETECTION_NO_TRIGGERS_EXECUTABLE peak_detection_no_triggers)

add_executable(${EXECUTABLE} main.cpp)

include_directories(QuadcopterPropellerDetector_lib)
add_subdirectory(QuadcopterPropellerDetector_lib)


# if HDF5_LIBRARIES is not set, set it to the HDF5::HDF5 target
if (NOT HDF5_LIBRARIES)
    set(HDF5_LIBRARIES hdf5 hdf5_cpp)
endif()

set(LIBRARIES QuadcopterPropellerDetector_lib MetavisionSDK::base MetavisionSDK::core MetavisionSDK::driver MetavisionSDK::ui Boost::program_options yaml-cpp Threads::Threads ${HDF5_LIBRARIES})

target_link_libraries(${EXECUTABLE} ${LIBRARIES})

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if (BUILD_IPU)
        target_link_libraries(${EXECUTABLE_IPU} QuadcopterPropellerDetector_lib MetavisionSDK::base MetavisionSDK::core MetavisionSDK::driver MetavisionSDK::ui Boost::program_options yaml-cpp Threads::Threads poplar poputil)
    endif()
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_subdirectory(Google_tests)
endif()
