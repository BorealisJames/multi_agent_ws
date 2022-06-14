cmake_minimum_required(VERSION 2.8.3)

project(distributed_formation)

add_compile_options(-std=c++11)

# Add custom cmake files folder
SET (PROJECT_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# This sets where to look for modules (e.g. "Find*.cmake" files)
SET (CMAKE_MODULE_PATH "${PROJECT_CMAKE_DIR}" "${CMAKE_MODULE_PATH}" )

SET(FORMATION2D
	src/Formation2D/Formation2D.h
	src/Formation2D/Formation2DWithoutYaw/Formation2DBase.h
	src/Formation2D/Formation2DWithoutYaw/Optimize2DFormation.h
	src/Formation2D/Formation2DWithoutYaw/Optimize2DFormation.cpp
	src/Formation2D/Formation2DWithoutYaw/Optimizer2DVarsConstrCost.h
	src/Formation2D/Formation2DWithoutYaw/Formation2DPoint1Agent.h
	src/Formation2D/Formation2DWithoutYaw/Formation2DPoint1Agent.cpp
	src/Formation2D/Formation2DWithoutYaw/Formation2DAbreast2Agents.h
	src/Formation2D/Formation2DWithoutYaw/Formation2DAbreast2Agents.cpp
	src/Formation2D/Formation2DWithoutYaw/Formation2DLine3Agents.h
	src/Formation2D/Formation2DWithoutYaw/Formation2DLine3Agents.cpp
	src/Formation2D/Formation2DWithoutYaw/Formation2DTri3Agents.h
	src/Formation2D/Formation2DWithoutYaw/Formation2DTri3Agents.cpp
	src/Formation2D/Formation2DWithYaw/Formation2DBase.h
	src/Formation2D/Formation2DWithYaw/Optimize2DFormation.h
	src/Formation2D/Formation2DWithYaw/Optimize2DFormation.cpp
	src/Formation2D/Formation2DWithYaw/Optimizer2DVarsConstrCost.h
	src/Formation2D/Formation2DWithYaw/Formation2DPoint1Agent.h
	src/Formation2D/Formation2DWithYaw/Formation2DPoint1Agent.cpp
	src/Formation2D/Formation2DWithYaw/Formation2DAbreast2Agents.h
	src/Formation2D/Formation2DWithYaw/Formation2DAbreast2Agents.cpp
	src/Formation2D/Formation2DWithYaw/Formation2DLine3Agents.h
	src/Formation2D/Formation2DWithYaw/Formation2DLine3Agents.cpp
	src/Formation2D/Formation2DWithYaw/Formation2DTri3Agents.h
	src/Formation2D/Formation2DWithYaw/Formation2DTri3Agents.cpp)

SET(FORMATION3D
	src/Formation3D/Formation3D.h
	src/Formation3D/Formation3DWithoutRot/Formation3DBase.h
	src/Formation3D/Formation3DWithoutRot/Optimize3DFormation.h
	src/Formation3D/Formation3DWithoutRot/Optimize3DFormation.cpp
	src/Formation3D/Formation3DWithoutRot/Optimizer3DVarsConstrCost.h
	src/Formation3D/Formation3DWithoutRot/Formation3DPoint1Agent.h
	src/Formation3D/Formation3DWithoutRot/Formation3DPoint1Agent.cpp
	src/Formation3D/Formation3DWithoutRot/Formation3DAbreast2Agents.h
	src/Formation3D/Formation3DWithoutRot/Formation3DAbreast2Agents.cpp
	src/Formation3D/Formation3DWithoutRot/Formation3DLine3Agents.h
	src/Formation3D/Formation3DWithoutRot/Formation3DLine3Agents.cpp
	src/Formation3D/Formation3DWithoutRot/Formation3DTri3Agents.h
	src/Formation3D/Formation3DWithoutRot/Formation3DTri3Agents.cpp
	src/Formation3D/Formation3DWithRot/Formation3DBase.h
	src/Formation3D/Formation3DWithRot/Optimize3DFormation.h
	src/Formation3D/Formation3DWithRot/Optimize3DFormation.cpp
	src/Formation3D/Formation3DWithRot/Optimizer3DVarsConstrCost.h
	src/Formation3D/Formation3DWithRot/Formation3DPoint1Agent.h
	src/Formation3D/Formation3DWithRot/Formation3DPoint1Agent.cpp
	src/Formation3D/Formation3DWithRot/Formation3DAbreast2Agents.h
	src/Formation3D/Formation3DWithRot/Formation3DAbreast2Agents.cpp
	src/Formation3D/Formation3DWithRot/Formation3DLine3Agents.h
	src/Formation3D/Formation3DWithRot/Formation3DLine3Agents.cpp
	src/Formation3D/Formation3DWithRot/Formation3DTri3Agents.h
	src/Formation3D/Formation3DWithRot/Formation3DTri3Agents.cpp
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DBase.h
	src/Formation3D/Formation3DWithOnlyYaw/Optimize3DFormation.h
	src/Formation3D/Formation3DWithOnlyYaw/Optimize3DFormation.cpp
	src/Formation3D/Formation3DWithOnlyYaw/Optimizer3DVarsConstrCost.h
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DPoint1Agent.h
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DPoint1Agent.cpp
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DAbreast2Agents.h
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DAbreast2Agents.cpp
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DLine3Agents.h
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DLine3Agents.cpp
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DTri3Agents.h
	src/Formation3D/Formation3DWithOnlyYaw/Formation3DTri3Agents.cpp)

		find_package(catkin REQUIRED
		COMPONENTS
		catkin_simple
		decomp_ros_utils
		ifopt
		roscpp
		sensor_msgs
		std_msgs
		pcl_conversions
		pcl_ros
		)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(eigen-cdd REQUIRED)
find_package(GTest REQUIRED)

catkin_package(INCLUDE_DIRS
				include
				${DECOMP_UTIL_INCLUDE_DIRS}
				LIBRARIES
				${PROJECT_NAME}_core
				CATKIN_DEPENDS
				catkin_simple
				decomp_ros_utils
				roscpp
				sensor_msgs
				std_msgs
				DEPENDS
				ifopt
				EIGEN3
				EIGEN_CDD
				pcl_conversions
				pcl_ros
)

include_directories(include
		${catkin_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${DECOMP_UTIL_INCLUDE_DIRS}
		${EIGEN_CDD_INCLUDE_DIRS}
		${GTEST_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_core
			src/ConvexHullOfRobotPosition/ConvexHullOfRobotPosition.h
			src/ConvexHullOfRobotPosition/ConvexHullOfRobotPosition.cpp
			src/DirectionOfMotion/DirectionOfMotion.h
			src/DirectionOfMotion/DirectionOfMotion.cpp
			src/GenerateConvexRegions/GenerateConvexRegions.h
			src/GenerateConvexRegions/GenerateConvexRegions.cpp
            src/DistributedMultiRobotFormation.h
            src/DistributedMultiRobotFormation.cpp
            src/DistributedMultiRobotFormationHandler.h
            src/DistributedMultiRobotFormationHandler.cpp
        	src/VirtualPositionAssignment/Matrix.h
        	src/VirtualPositionAssignment/Munkres.h
        	src/VirtualPositionAssignment/Munkres.cpp
        	src/VirtualPositionAssignment/VirtualPositionAssignment.h
        	src/VirtualPositionAssignment/VirtualPositionAssignment.cpp
            src/Common/Common.h
			${FORMATION2D}
			${FORMATION3D}
            src/PhaseSynchronizer/PhaseSynchronizer.h
            src/PhaseSynchronizer/PhaseSynchronizer.cpp
			src/ProcessPointCloud/ProcessPointCloud.h
			src/ProcessPointCloud/ProcessPointCloud.cpp
        	src/FollowMeGoalGenerator/FollowMeGoalGenerator.h
        	src/FollowMeGoalGenerator/FollowMeGoalGenerator.cpp)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME} src/main.cpp)

# Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_core
        	${catkin_EXPORTED_TARGETS})

# Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core ${catkin_LIBRARIES} ${EIGEN_CDD_LIBRARIES})

catkin_add_gtest(RunTest
				src/Test/RunTests.cpp
				src/Test/ConvexHullOfRobotPositionTest.cpp
				src/Test/DHBATest.cpp
				src/Test/DirectionOfMotionTest.cpp
				src/Test/FollowMeGoalGeneratorTest.cpp
				src/Test/GenerateConvexRegionsTest.cpp
        		src/Test/Optimize2DFormationWithoutYawTest.cpp
				src/Test/Optimize2DFormationWithYawTest.cpp
				src/Test/Optimize3DFormationWithOnlyYawTest.cpp
				src/Test/Optimize3DFormationWithoutRotTest.cpp
				src/Test/Optimize3DFormationWithRotTest.cpp
				src/Test/ProcessPointCloudTest.cpp)
target_link_libraries(RunTest ${PROJECT_NAME}_core ${catkin_LIBRARIES} ${EIGEN_CDD_LIBRARIES})