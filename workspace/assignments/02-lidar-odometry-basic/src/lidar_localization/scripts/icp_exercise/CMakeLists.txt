cmake_minimum_required (VERSION 3.4)

project(icp_test)

if(NOT BUILD_QNX)
set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3)
find_package(Ceres REQUIRED)
message(STATUS "EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
message(STATUS "CERES_INCLUDE_DIRS=${CERES_INCLUDE_DIRS}")
endif ()

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  /home/levin/workspace/tools/ceres-solver/internal
  ./include
  ../
)

add_definitions( -fPIC)	
add_library(icp STATIC src/icp.cpp src/CeresDebugInfo.cpp)
target_link_libraries(icp  ${CERES_LIBRARIES} glog gflags)


add_executable(icp_test  src/main.cpp )				
target_link_libraries(icp_test  icp glog gflags ${PCL_LIBRARIES})	

			   	   
			   
