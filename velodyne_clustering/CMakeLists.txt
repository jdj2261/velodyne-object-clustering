cmake_minimum_required(VERSION 3.10)

# Create Project
project(velodyne_cluster)

# Require c++11
set(CMake_CXX_COMPILER g++)
set(CMake_CXX_STANDARD 11)
set(CMake_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions("-Wall -Wextra -std=c++11")

# Find Packages
find_package(PCL 1.8 REQUIRED)
find_package(Threads REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.65.1 COMPONENTS system data_time regex)

set(HAVE_BOOST)
if(Boost_FOUND)
                set(HAVE_BOOST "-DHAVE_BOOST")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})
find_package(PCAP)

set(HAVE_PCAP)
if(PCAP_FOUND)
                set(HAVE_PCAP "-DHAVE_PCAP")
endif()

# Set Properties
if(Boost_FOUND OR PCAP_FOUND)
                # Add_Definitions
                add_definitions( ${Boost_LIB_DIAGNOSTIC_DEFINITIONS} )
                add_definitions( ${HAVE_BOOST} )
                add_definitions( ${HAVE_PCAP} )

                # Additional Include Directories
                include_directories(include)
                include_directories(${Boost_INCLUDE_DIRS})
                include_directories(${PCAP_INCLUDE_DIRS})

                # Additional Library Directories
                link_directories(${Boost_USE_MULTITHREADED})

                file(GLOB SOURCE_FILES
                    include/velodyne_driver/*.h
                    include/velodyne_pointcloud/*.h
                    src/driver/*.cpp
                    src/main.cpp)
                add_executable(${CMAKE_PROJECT_NAME} ${SOURCE_FILES})
		
                # Additional Dependencies
                target_link_libraries(${CMAKE_PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
                target_link_libraries(${CMAKE_PROJECT_NAME} ${Boost_LIBRARIES})
                target_link_libraries(${CMAKE_PROJECT_NAME} ${PCAP_LIBRARIES})
else()
                message( WARNING "This project need at least either Boost or PCAP Lib.")
endif()
