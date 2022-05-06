cmake_minimum_required(VERSION 2.8.3)

project(borealis_pose_tf_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        tf
        geometry_msgs)

catkin_package(INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}_core
        CATKIN_DEPENDS
        roscpp
        tf
        geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_core
        src/HRITfManager.cpp)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME} src/main.cpp)

# Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_core
        ${catkin_EXPORTED_TARGETS})

# Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core ${catkin_LIBRARIES})
