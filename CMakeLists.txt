cmake_minimum_required(VERSION 3.5)
project(patrolling_sim_ros2)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stage_ros2 REQUIRED)
find_package(p3dx_description_ros REQUIRED)
find_package(fake_localization_ros2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# service definition used by GoToStartPos service
set(srv_files
  "srv/GoToStartPosSrv.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

# Declare a cpp library
add_library(PatrolAgent 
            src/PatrolAgent.cpp 
            src/getgraph.cpp 
            src/algorithms.cpp
            src/config.cpp)

#target_include_directories(PatrolAgent PUBLIC
#            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#            "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
#            "$<INSTALL_INTERFACE:include>")      

ament_target_dependencies(PatrolAgent
  rclcpp ament_index_cpp nav2_msgs rclcpp_action tf2_ros nav_msgs std_srvs tf2_geometry_msgs tf2)

#install(
#  TARGETS PatrolAgent EXPORT PatrolAgent
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# Declare a cpp library
add_library(SSIPatrolAgent 
            src/SSIPatrolAgent.cpp
            src/algorithms.cpp)
            
ament_target_dependencies(SSIPatrolAgent rclcpp nav2_msgs rclcpp_action tf2_ros)

include_directories(src)
#ament_export_include_directories(include)
#ament_export_libraries(PatrolAgent)
#ament_export_targets(PatrolAgent)



## Declare a cpp executable
add_executable(Conscientious_Reactive src/Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
##target_link_libraries(Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Conscientious_Reactive rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(Conscientious_Reactive PatrolAgent)

## Declare a cpp executable
add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
##target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Heuristic_Conscientious_Reactive rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent)
#  
## Declare a cpp executable 
add_executable(Conscientious_Cognitive src/Conscientious_Cognitive_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(Conscientious_Cognitive PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Conscientious_Cognitive rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(Conscientious_Cognitive PatrolAgent)

## Declare a cpp executable 
add_executable(Cyclic src/Cyclic_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(Cyclic PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Cyclic rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(Cyclic PatrolAgent)

## Declare a cpp executable 
add_executable(MSP src/MSP_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(MSP PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(MSP rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(MSP PatrolAgent)

## Declare a cpp executable 
add_executable(GBS src/GBS_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(GBS PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(GBS rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(GBS PatrolAgent)

## Declare a cpp executable 
add_executable(SEBS src/SEBS_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(SEBS PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(SEBS rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(SEBS PatrolAgent)

## Declare a cpp executable 
add_executable(CBLS src/CBLS_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(CBLS PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(CBLS rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(CBLS PatrolAgent)

## Declare a cpp executable 
add_executable(Random src/Random_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(Random PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Random rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs std_msgs)
target_link_libraries(Random PatrolAgent)

## Declare a cpp executable 
add_executable(DTAGreedy src/DTAGreedy_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(DTAGreedy PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(DTAGreedy rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs std_msgs)
target_link_libraries(DTAGreedy PatrolAgent)

## Declare a cpp executable 
add_executable(DTASSI src/DTASSI_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(DTASSI PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(DTASSI rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs std_msgs)
target_link_libraries(DTASSI PatrolAgent SSIPatrolAgent)

## Declare a cpp executable 
add_executable(DTASSIPart src/DTASSIPart_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(DTASSIPart PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(DTASSIPart rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs std_msgs)
target_link_libraries(DTASSIPart PatrolAgent SSIPatrolAgent )
 
# results and monitor Node
add_executable(monitor src/monitor.cpp src/getgraph.cpp)
ament_target_dependencies(monitor
  rclcpp rclcpp_action nav2_msgs tf2_ros std_msgs ament_index_cpp
)
# the following lines are required to use in this node the new interface defined in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(monitor "${cpp_typesupport_target}")

add_executable(GoToStartPos src/GoToStartPos.cpp)
ament_target_dependencies(GoToStartPos
  rclcpp rclcpp_action nav2_msgs tf2_ros tf2_geometry_msgs geometry_msgs
)
# the following line is required to use in this node the new interface defined in the same package
target_link_libraries(GoToStartPos "${cpp_typesupport_target}")

## Declare a cpp executable 
add_executable(twist_stamped_to_twist src/twist_stamped_to_twist.cpp)
ament_target_dependencies(twist_stamped_to_twist rclcpp geometry_msgs)



#############
## Install ##
#############

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(nav2_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(stage_ros2)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)


# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS monitor Conscientious_Reactive Heuristic_Conscientious_Reactive
  Conscientious_Cognitive Cyclic MSP GBS SEBS CBLS Random DTAGreedy
  DTASSI DTASSIPart GoToStartPos twist_stamped_to_twist DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY MSP DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION $ENV{HOME}/.gazebo/models/${PROJECT_NAME})


# # Install bash scripts
# install(PROGRAMS
#   stage_multi_robot_patrolling.py
#   gazebo_multi_robot_patrolling.py
#   stop_experiment.sh
#   run_exp.sh
#   abort_experiment.sh
#   kill_all.sh
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()