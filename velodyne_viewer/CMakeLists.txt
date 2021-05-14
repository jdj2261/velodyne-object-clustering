cmake_minimum_required( VERSION 3.6 )

# Create Project
project( velodyne_viewer )
add_definitions(-std=c++11)
# Require C++11
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})



# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "velodyne_viewer" )

# Find Packages

# Try to find Point Cloud Library
find_package(PCL 1.8 REQUIRED)

# Find Package Thread
find_package( Threads REQUIRED )

# Find Package Boost
set( BOOST_ROOT "C:/Program Files/boost" )
set( Boost_USE_STATIC_LIBS ON ) # Static Link Libraries ( libboost_* )
set( Boost_USE_MULTITHREADED ON ) # Multi Thread Libraries ( *-mt-* )
set( Boost_USE_STATIC_RUNTIME OFF ) # Static Runtime Libraries ( *-s* )
find_package( Boost COMPONENTS system date_time regex )

# If Direct Capture from Sensors, VelodyneCapture are required Boost.Asio.
# Please Define HAVE_BOOST in Preprocessor.
set( HAVE_BOOST )
if( Boost_FOUND )
  set( HAVE_BOOST "-DHAVE_BOOST" )
endif()

# Find Package PCAP
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
find_package( PCAP )

# Disable sleep while reading a pcap
# Is is useful if you have to parse the result of a pcap faster than real
# time.

OPTION( HAVE_FAST_PCAP "Avoid sleeping whilst reading PCAP files " OFF )
if( HAVE_FAST_PCAP )
  add_definitions( -DHAVE_FAST_PCAP )
endif()

# If Capture from PCAP Files, VelodyneCapture are required PCAP.
# Please Define HAVE_PCAP in Preprocessor.
set( HAVE_PCAP )
if( PCAP_FOUND )
  set( HAVE_PCAP "-DHAVE_PCAP" )
endif()

# Use GPS timestamps insead of Unix epoch
# This is specially useful while reading PCAP files.
OPTION( HAVE_GPSTIME "Use GPS timestamps instead of Unix " OFF )
if( HAVE_GPSTIME )
  add_definitions( -DHAVE_GPSTIME )
endif()

# Use millimeters instead of the default, centimeters.
OPTION( USE_MILLIMETERS "Use millimeters instead of centimeters " OFF )
if( USE_MILLIMETERS )
  add_definitions( -DUSE_MILLIMETERS )
endif()

# Push data packets as soon as possible, if ndef the drivers push the 
# whole frame
OPTION( PUSH_SINGLE_PACKETS  "Push data packets to the stack instead of frames" OFF )
if( PUSH_SINGLE_PACKETS )
  add_definitions( -DPUSH_SINGLE_PACKETS )
endif()

# Don't push null returns to the lases queue
OPTION( NO_EMPTY_RETURNS "Discard empty returns before pushing them to the laser queue" OFF )
if( NO_EMPTY_RETURNS )
  add_definitions( -DNO_EMPTY_RETURNS )
endif()

# Find Package OpenCV
set( OpenCV_DIR "C:/Program Files/opencv/build" )
set( OpenCV_STATIC OFF )
find_package( OpenCV REQUIRED )

# Required Viz Module
if( OpenCV_FOUND )
  if( NOT "opencv_viz" IN_LIST OpenCV_LIBS )
    message( FATAL_ERROR "not found opencv_viz module." )
  endif()
endif()

if( OpenCV_STATIC AND MSVC )
  foreach( flag_var
      CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
      CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO )
    if( ${flag_var} MATCHES "/MD" )
      string( REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}" )
    endif()
  endforeach()
endif()

# Set Properties
if( OpenCV_FOUND AND ( PCAP_FOUND OR Boost_FOUND ) )
  # Additional Include Directories
  include_directories( include ${Boost_INCLUDE_DIRS} ${PCAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  # include_directories( include/process ${Boost_INCLUDE_DIRS} ${PCAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
  # include_directories( include/render ${Boost_INCLUDE_DIRS} ${PCAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
  # include_directories( include/velodyne ${Boost_INCLUDE_DIRS} ${PCAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  include_directories( src ${Boost_INCLUDE_DIRS} ${PCAP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

  # Add Definitions
  add_definitions( ${Boost_LIB_DIAGNOSTIC_DEFINITIONS} )
  add_definitions( ${HAVE_BOOST} )
  add_definitions( ${HAVE_PCAP} )
  add_definitions(${PCL_DEFINITIONS})

  # Additional Library Directories
  link_directories( ${Boost_LIBRARY_DIRS} )
  link_directories( ${OpenCV_LIB_DIR} )
  link_directories(${PCL_LIBRARY_DIRS})
  link_directories(include)

  file(GLOB SOURCE_FILES include/*.hpp include/process/*.hpp include/render/*.hpp include/velodyne/*.hpp src/*.cpp src/process/*.cpp src/render/*.cpp)
  add_executable( ${CMAKE_PROJECT_NAME} ${SOURCE_FILES} )

  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

  # Additional Dependencies
  target_link_libraries( ${CMAKE_PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} )
  target_link_libraries( ${CMAKE_PROJECT_NAME} ${Boost_LIBRARIES} )
  target_link_libraries( ${CMAKE_PROJECT_NAME} ${PCAP_LIBRARIES} )
  target_link_libraries( ${CMAKE_PROJECT_NAME} ${OpenCV_LIBS} )
  target_link_libraries( ${CMAKE_PROJECT_NAME} ${PCL_LIBRARIES} )
  target_link_libraries( ${CMAKE_PROJECT_NAME} ${catkin_LIBRARIES} )

else()
  message( WARNING "velodyne_viewer need at least either Boost or PCAP." )
endif()
