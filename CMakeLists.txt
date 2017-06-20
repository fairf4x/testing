cmake_minimum_required(VERSION 2.8.3)
project(testing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	nav_msgs
	std_msgs
	geometry_msgs
	moveit_msgs
	roscpp
	rospy
	tf
	message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools)

find_package(Eigen3 REQUIRED)

add_service_files(DIRECTORY srv
    FILES
    GetEigenValue.srv
    ComputeTransformationMatrix.srv
    GetRobotTrajectoryFromPath.srv
#    NOINSTALL
    )

  generate_messages(DEPENDENCIES std_msgs geometry_msgs moveit_msgs nav_msgs)
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    moveit_msgs
    roscpp)

include_directories(
	${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})

add_executable(frodopathy src/frodopath.cpp)
add_executable(tftodo src/tftodo.cpp)
add_executable(ultimetry src/ultimetry.cpp)
add_executable(eigenserviceprovider src/eigenserviceprovider.cpp)
add_executable(servicetest src/servicetest.cpp)
add_executable(cmdvelrepeater src/cmdvelrepeater.cpp)
add_executable(cmdvellimiter src/cmdvellimiter.cpp)
add_executable(transodom src/transodom.cpp)
add_executable(robottrajectory src/robottrajectoryfrompathservice.cpp)
add_executable(backtracker src/backtracker.cpp)
target_link_libraries(frodopathy ${catkin_LIBRARIES})
target_link_libraries(ultimetry ${catkin_LIBRARIES})
target_link_libraries(eigenserviceprovider ${catkin_LIBRARIES})
target_link_libraries(servicetest ${catkin_LIBRARIES})
target_link_libraries(tftodo ${catkin_LIBRARIES})
target_link_libraries(cmdvelrepeater ${catkin_LIBRARIES})
target_link_libraries(cmdvellimiter ${catkin_LIBRARIES})
target_link_libraries(transodom ${catkin_LIBRARIES})
target_link_libraries(robottrajectory ${catkin_LIBRARIES})
target_link_libraries(backtracker ${catkin_LIBRARIES})
#add_dependencies(testingmessages testing_generate_messages_cpp)
add_dependencies(frodopathy frodopathy_gencpp)
add_dependencies(ultimetry ultimetry_gencpp)
add_dependencies(tftodo ultimetry_gencpp)
add_dependencies(cmdvelrepeater cmdvelrepeater_gencpp)
add_dependencies(cmdvellimiter cmdvellimiter_gencpp)
add_dependencies(transodom transodom_gencpp)
add_dependencies(backtracker backtracker_gencpp)
add_dependencies(robottrajectory robottrajectory_gencpp)
add_dependencies(robottrajectory testing_generate_messages_cpp)
add_dependencies(eigenserviceprovider eigenserviceprovider_gencpp)
add_dependencies(eigenserviceprovider testing_generate_messages_cpp)
add_dependencies(servicetest eigenserviceprovider_gencpp)
add_dependencies(servicetest testing_generate_messages_cpp)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
