cmake_minimum_required(VERSION 2.8.3)

project(distributed_global_path_planner)

add_compile_options(-std=c++11)

# Add custom cmake files folder
SET (PROJECT_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# This sets where to look for modules (e.g. "Find*.cmake" files)
SET (CMAKE_MODULE_PATH "${PROJECT_CMAKE_DIR}" "${CMAKE_MODULE_PATH}" )

SET(JPS
	src/JumpPointSearch/jps3d/src/distance_map_planner/distance_map_planner.cpp
	src/JumpPointSearch/jps3d/src/distance_map_planner/graph_search.cpp
	src/JumpPointSearch/jps3d/src/jps_planner/graph_search.cpp
	src/JumpPointSearch/jps3d/src/jps_planner/jps_planner.cpp)


find_package(catkin REQUIRED
			COMPONENTS
			catkin_simple
			geometry_msgs
			roscpp
			sensor_msgs
			visualization_msgs
			std_msgs)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

catkin_package(INCLUDE_DIRS
				include
				src/JumpPointSearch/jps3d/include
				LIBRARIES
				${PROJECT_NAME}_core
				CATKIN_DEPENDS
				catkin_simple
				geometry_msgs
				roscpp
				sensor_msgs
				std_msgs
				visualization_msgs
				DEPENDS
				EIGEN3)

include_directories(include
					src/JumpPointSearch/jps3d/include
					${catkin_INCLUDE_DIRS}
					${EIGEN3_INCLUDE_DIRS}
					${GTEST_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_core
			${JPS}
			src/DistributedGlobalPathPlanner.h
			src/DistributedGlobalPathPlanner.cpp
			src/DistributedGlobalPathPlannerHandler.h
			src/DistributedGlobalPathPlannerHandler.cpp
			src/GoTherePathTracker/GoTherePathTracker.h
			src/GoTherePathTracker/GoTherePathTracker.cpp
			src/JumpPointSearch/PathPlanning2DHandle.h
			src/JumpPointSearch/PathPlanning2DHandle.cpp
			src/JumpPointSearch/PathPlanning3DHandle.h
			src/JumpPointSearch/PathPlanning3DHandle.cpp
			src/JumpPointSearch/PathPlanningParams.h
			src/JumpPointSearch/read_map.hpp
			src/PhaseSynchronizer/PhaseSynchronizer.h
			src/PhaseSynchronizer/PhaseSynchronizer.cpp
			src/ProcessPointCloud/ProcessPointCloud.h
			src/ProcessPointCloud/ProcessPointCloud.cpp)

# Each node in the package must be declared like this
#add_executable(${PROJECT_NAME}
#				src/main.cpp)

# Add dependencies to exported targets, like ROS msgs or srvs
#add_dependencies(${PROJECT_NAME}
#					${PROJECT_NAME}_core
#					${catkin_EXPORTED_TARGETS})

# Specify libraries to link executable targets against
#target_link_libraries(${PROJECT_NAME}
#						${PROJECT_NAME}_core
#						${catkin_LIBRARIES})

catkin_add_gtest(RunTest
				src/Test/RunTests.cpp)

target_link_libraries(RunTest 
						${PROJECT_NAME}_core
						${catkin_LIBRARIES})
