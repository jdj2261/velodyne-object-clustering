cmake_minimum_required(VERSION 3.10)

# Create Project
project(vlp16-viewer)
add_definitions("-Wall -Wextra -std=c++11")

# Require c++11
set(CMake_CXX_COMPILER g++)
set(CMake_CXX_STANDARD 11)
set(CMake_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(HAVE_BOOST)

# Find Packages
find_package(PCL 1.7 REQUIRED)
find_package(Threads REQUIRED)


if(Boost_FOUND)
    set(HAVE_BOOST "-DHAVE_BOOST")
endif()
find_package(Boost COMPONENTS system data_time regex)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})
find_package(PCAP)

set(HAVE_PCAP)
if(PCAP_FOUND)
    set(HAVE_PCAP "-DHAVE_PCAP")
endif()

# Set Properties
if(Boost_FOUND OR PCAP_FOUND)
    # Add_Definitions
    add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
    add_definitions(${HAVE_BOOST})
    add_definitions(${HAVE_PCAP})
    add_definitions(${PCL_DEFINITIONS})

    # Additional Include Directories
    include_directories(include)
    include_directories(${Boost_INCLUDE_DIRS})
    include_directories(${PCAP_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})

    # Additional Library Directories
    link_directories(${Boost_USE_MULTITHREADED})
    link_directories(${PCL_LIBRARY_DIRS})

#    file(GLOB SOURCE_FILES
#        include/*.h
#        include/velodyne_driver/*.h
#        include/velodyne_pointcloud/*.h
#        include/velodyne_pcl/*.h
#        src/velodyne_driver/*.cpp
#        src/velodyne_pointcloud/*.cpp
#        src/velodyne_pcl/*.cpp)

    # add Libraries
    add_library(VelodyneDriver SHARED
        include/velodyne_driver/driver.h
        include/velodyne_driver/input.h
        src/velodyne_driver/driver.cpp
        src/velodyne_driver/input.cpp)
    target_link_libraries(VelodyneDriver)

    add_library(VelodynePointCloud SHARED
        include/velodyne_pointcloud/calibration.h
        include/velodyne_pointcloud/datacontainerbase.h
        include/velodyne_pointcloud/point_types.h
        include/velodyne_pointcloud/pointcloudXYZIR.h
        include/velodyne_pointcloud/rawdata.h
        include/velodyne_pointcloud/transform.h
        src/velodyne_pointcloud/calibration.cpp
        src/velodyne_pointcloud/pointcloudXYZIR.cpp
        src/velodyne_pointcloud/rawdata.cpp
        src/velodyne_pointcloud/transform.cpp)
    target_link_libraries(VelodynePointCloud)

    add_library(VelodynePCL SHARED
        include/velodyne_pcl/viewer.h
        include/velodyne_pcl/timer.h
        src/velodyne_pcl/viewer.cpp)

#    target_link_libraries(VelodynePCL)

    # make exec file
    add_executable(${CMAKE_PROJECT_NAME} src/main.cpp)

    # Additional Dependencies
    target_link_libraries(${CMAKE_PROJECT_NAME}
        VelodyneDriver
        VelodynePointCloud
        VelodynePCL
        )
    target_link_libraries(${CMAKE_PROJECT_NAME} ${Boost_LIBRARIES})
    target_link_libraries(${CMAKE_PROJECT_NAME} ${PCAP_LIBRARIES})
    target_link_libraries(${CMAKE_PROJECT_NAME} ${PCL_LIBRARIES})


    # for Test
    add_executable(ViewerTest tests/viewerTest.cpp)
    target_link_libraries(ViewerTest
        VelodyneDriver
        VelodynePointCloud
        VelodynePCL
        )
    target_link_libraries(ViewerTest ${Boost_LIBRARIES})
    target_link_libraries(ViewerTest ${PCAP_LIBRARIES})
    target_link_libraries(ViewerTest ${PCL_LIBRARIES})
    set_target_properties(ViewerTest
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY
        "${CMAKE_BINARY_DIR}/tests")
else()
    message( WARNING "This project need at least either Boost or PCAP Lib.")
endif()
