cmake_minimum_required(VERSION 3.5)
project(hrc_task_manager)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# --- Interface generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RunBehaviorTree.srv"
  # DEPENDENCIES std_msgs  # uncomment only if your .srv uses std_msgs types
)

# Humble preferred way to link typesupport
rosidl_get_typesupport_target(rosidl_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- ServiceUtils lib (implements ServiceUtils::call_service, delay_ms, etc.) ---
add_library(service_utils src/service_utils.cpp)
ament_target_dependencies(service_utils rclcpp std_srvs)

# Make sure it’s installed
install(TARGETS service_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# --- Custom BehaviorTree node sources (these call ServiceUtils) ---
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
)

# Build node sources into a library
add_library(hrc_bt_nodes STATIC ${NODE_SRCS})
ament_target_dependencies(hrc_bt_nodes
  rclcpp
  behaviortree_cpp
  std_msgs
  std_srvs
)

# IMPORTANT: if your node .cpp files include and call ServiceUtils,
# also link service_utils here (or at least in each consuming executable).
# Linking here ensures any target that links hrc_bt_nodes gets ServiceUtils too.
target_link_libraries(hrc_bt_nodes service_utils)

# --- bt_server executable ---
add_executable(bt_server src/bt_server.cpp)
ament_target_dependencies(bt_server rclcpp behaviortree_cpp std_msgs std_srvs)
target_link_libraries(bt_server
  ${rosidl_typesupport_target}
  hrc_bt_nodes         # provides your BT nodes
  service_utils        # provide ServiceUtils used inside nodes (safe even if linked via hrc_bt_nodes)
)
install(TARGETS bt_server DESTINATION lib/${PROJECT_NAME})

# --- bt_client executable ---
add_executable(bt_client src/bt_client.cpp)
ament_target_dependencies(bt_client rclcpp)
target_link_libraries(bt_client ${rosidl_typesupport_target})
install(TARGETS bt_client DESTINATION lib/${PROJECT_NAME})

# --- main2 (existing BT runner) ---
add_executable(main2 src/main2.cpp)
target_link_libraries(main2
  hrc_bt_nodes
  service_utils
)
ament_target_dependencies(main2
  rclcpp
  behaviortree_cpp
  std_msgs
  std_srvs
)
install(TARGETS main2 DESTINATION lib/${PROJECT_NAME})

# --- dummy_node ---
add_executable(dummy_node src/dummy_node.cpp)
ament_target_dependencies(dummy_node rclcpp std_srvs)
install(TARGETS dummy_node DESTINATION lib/${PROJECT_NAME})

# --- tests / ament ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
