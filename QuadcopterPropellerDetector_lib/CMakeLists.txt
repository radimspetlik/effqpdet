project(QuadcopterPropellerDetectorLib)

if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(BOOST_ROOT  "c:/Program Files/Prophesee/third_party")
    set(Boost_INCLUDE_DIR "c:/Program Files/Prophesee/third_party/include")
    set(Boost_LIBRARY_DIR "c:/Program Files/Prophesee/third_party/lib")
    set(Boost_COMPILER "-vc140")
    set(Boost_ARCHITECTURE "-mt")

    set(TIFF_INCLUDE_DIR "c:/Program Files/Prophesee/third_party/include")
    set(TIFF_LIBRARY "c:/Program Files/Prophesee/third_party/lib")

    set(HDF5_INCLUDE_DIRS "c:/Users/radio/.vcpkg-clion/vcpkg/installed/x64-windows/include")
    find_package(HDF5 CONFIG REQUIRED COMPONENTS C CXX)
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(HDF5 REQUIRED COMPONENTS C CXX)
    include_directories("/usr/include/hdf5/serial")
endif()

find_package(yaml-cpp REQUIRED)
find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)
find_package(Protobuf REQUIRED)

file(GLOB protos "proto/*.proto")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${protos})

# if HDF5_LIBRARIES is not set, set it to the HDF5::HDF5 target
if (NOT HDF5_LIBRARIES)
    set(HDF5_LIBRARIES hdf5 hdf5_cpp)
endif()

set(HEADER_FILES
        crc.h
        Detector.h
        types.h
        TensorBoardLogger.h
        ${PROTO_HDRS}
)

set(SOURCE_FILES
        crc.cc
        Detector.cpp
        TensorBoardLogger.cpp
        ${PROTO_SRCS}
)

add_library(QuadcopterPropellerDetector_lib STATIC ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(QuadcopterPropellerDetector_lib
        PUBLIC
        ${HDF5_LIBRARIES}
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROTOBUF_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
target_link_libraries(QuadcopterPropellerDetector_lib yaml-cpp MetavisionSDK::core MetavisionSDK::driver MetavisionSDK::ui  Boost::timer ${Protobuf_LIBRARIES}  ${HDF5_LIBRARIES})
