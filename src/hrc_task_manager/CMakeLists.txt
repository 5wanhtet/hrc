cmake_minimum_required(VERSION 3.5)
project(hrc_task_manager)

# Find dependencies here
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include)

# Functions for client node (initialisation, sending service command, waiting for service etc)
add_library(service_utils src/service_utils.cpp)
ament_target_dependencies(service_utils
  rclcpp
  std_srvs
)

# List all your node source files here
set(NODE_SRCS
  src/condition/VerifyPosition.cpp
  src/condition/CheckTool.cpp
  src/action/AskForHelp.cpp
  src/action/ChangeTool.cpp
  src/action/DetachTool.cpp
  src/action/DetectObject.cpp
  src/action/MoveTo.cpp
  src/action/PickObject.cpp
  src/action/PlaceObject.cpp
  src/action/Unbolt.cpp
  src/action/AttachTool.cpp
  src/action/AlignToolWithTarget.cpp
  # Add other node .cpp files here
)

# Executable 2: main2
add_executable(main2
  src/main2.cpp
  ${NODE_SRCS}
)
target_link_libraries(main2 service_utils)
ament_target_dependencies(main2
  rclcpp
  behaviortree_cpp
  std_msgs
  std_srvs
)
install(TARGETS main2
  DESTINATION lib/${PROJECT_NAME}
)

# Executable 3: dummy node
add_executable(dummy_node src/dummy_node.cpp)
ament_target_dependencies(dummy_node rclcpp std_srvs)
install(TARGETS dummy_node DESTINATION lib/${PROJECT_NAME})

# Executable 4: server client
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
