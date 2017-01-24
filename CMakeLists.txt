cmake_minimum_required(VERSION 2.8.3)
project(merbots_ibvs_recovery)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	auv_msgs
	std_msgs
  std_srvs
  message_filters
  merbots_ibvs
  control)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(merbots_ibvs_recovery src/merbots_ibvs_recovery.cpp)
target_link_libraries(merbots_ibvs_recovery ${catkin_LIBRARIES})
add_dependencies(merbots_ibvs_recovery ${catkin_EXPORTED_TARGETS})
