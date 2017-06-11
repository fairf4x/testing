cmake_minimum_required(VERSION 2.8.3)
project(testing)

find_package(catkin REQUIRED COMPONENTS 
	nav_msgs
	std_msgs
	geometry_msgs
	roscpp
	rospy
	tf
	message_generation)

find_package(Eigen3 REQUIRED)

add_service_files(DIRECTORY srv
    FILES
    GetEigenValue.srv
    ComputeTransformationMatrix.srv
#    NOINSTALL
    )

  generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(testing
#   src/${PROJECT_NAME}/testing.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(testing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(testing_node src/testing_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(testing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(testing_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS testing testing_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_testing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

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
target_link_libraries(frodopathy ${catkin_LIBRARIES})
target_link_libraries(ultimetry ${catkin_LIBRARIES})
target_link_libraries(eigenserviceprovider ${catkin_LIBRARIES})
target_link_libraries(servicetest ${catkin_LIBRARIES})
target_link_libraries(tftodo ${catkin_LIBRARIES})
target_link_libraries(cmdvelrepeater ${catkin_LIBRARIES})
target_link_libraries(cmdvellimiter ${catkin_LIBRARIES})
#add_dependencies(testingmessages testing_generate_messages_cpp)
add_dependencies(frodopathy frodopathy_gencpp)
add_dependencies(ultimetry ultimetry_gencpp)
add_dependencies(tftodo ultimetry_gencpp)
add_dependencies(cmdvelrepeater cmdvelrepeater_gencpp)
add_dependencies(cmdvellimiter cmdvellimiter_gencpp)
add_dependencies(eigenserviceprovider eigenserviceprovider_gencpp)
add_dependencies(eigenserviceprovider testing_generate_messages_cpp)
add_dependencies(servicetest eigenserviceprovider_gencpp)
add_dependencies(servicetest testing_generate_messages_cpp)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
