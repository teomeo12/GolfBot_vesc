cmake_minimum_required(VERSION 3.8)
project(vesc_driver)

# Set minimum C++ standard to C++14
if(NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Threads)

###########
## Build ##
###########

# node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_driver::VescDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)

# # Include directories
# include_directories(
#   include
#   ${Boost_INCLUDE_DIRS}
# )

# # Node executable
# add_executable(vesc_driver_node 
#   src/vesc_driver_node.cpp
#   src/vesc_driver.cpp
#   src/vesc_interface.cpp
#   src/vesc_packet.cpp
#   src/vesc_packet_factory.cpp
# )
# ament_target_dependencies(vesc_driver_node
#   rclcpp
#   std_msgs
#   vesc_msgs
#   serial
# )
# install(TARGETS vesc_driver_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Nodelet (Component) library
# add_library(vesc_driver_nodelet SHARED 
#   src/vesc_driver_nodelet.cpp
#   src/vesc_driver.cpp
#   src/vesc_interface.cpp
#   src/vesc_packet.cpp
#   src/vesc_packet_factory.cpp
# )
# ament_target_dependencies(vesc_driver_nodelet
#   rclcpp
#   rclcpp_components
#   std_msgs
#   vesc_msgs
#   pluginlib
#   serial
# )
# rclcpp_components_register_nodes(vesc_driver_nodelet "vesc_driver::VescDriverNodelet")
# install(TARGETS vesc_driver_nodelet
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install headers
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION include/${PROJECT_NAME}/)

# # Install other files (e.g., XML, launch)
# install(FILES vesc_driver_nodelet.xml
#         DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY launch/
#         DESTINATION share/${PROJECT_NAME}/launch)

# # Package export
# ament_export_dependencies(rclcpp rclcpp_components std_msgs vesc_msgs pluginlib serial)
# ament_package()
