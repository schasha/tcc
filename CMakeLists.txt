cmake_minimum_required(VERSION 3.14)
project(semantic_gateway VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── Message generation ────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SmartData.msg"
  DEPENDENCIES std_msgs
)

# Helper to link generated interfaces to a target (ROS 2 Humble+)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── Common dependency list ────────────────────────────────────────────────────
set(NODE_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  rcpputils
  ament_index_cpp
)

# ── Shared library: gateway core (config loader + field access + decomposer) ──
add_library(gateway_core SHARED
  src/config_loader.cpp
  src/field_access.cpp
)
target_include_directories(gateway_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(gateway_core ${NODE_DEPS})
target_link_libraries(gateway_core
  "${cpp_typesupport_target}"
  yaml-cpp
)

# ── gateway_node ──────────────────────────────────────────────────────────────
add_executable(gateway_node src/gateway_node.cpp)
target_include_directories(gateway_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(gateway_node ${NODE_DEPS})
target_link_libraries(gateway_node "${cpp_typesupport_target}" gateway_core)

# ── validator_node ────────────────────────────────────────────────────────────
add_executable(validator_node src/validator_node.cpp)
target_include_directories(validator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(validator_node ${NODE_DEPS})
target_link_libraries(validator_node "${cpp_typesupport_target}" gateway_core)

# ── demo_publisher_node ───────────────────────────────────────────────────────
add_executable(demo_publisher_node src/demo_publisher_node.cpp)
target_include_directories(demo_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(demo_publisher_node ${NODE_DEPS})
target_link_libraries(demo_publisher_node "${cpp_typesupport_target}")

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS gateway_node validator_node demo_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS gateway_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY include/
  DESTINATION include
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto   REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests: marshaling/unmarshaling, SI units, latency bounds
  ament_add_gtest(test_smartdata_marshaling
    test/test_smartdata_marshaling.cpp
  )
  target_include_directories(test_smartdata_marshaling PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_smartdata_marshaling ${NODE_DEPS})
  target_link_libraries(test_smartdata_marshaling "${cpp_typesupport_target}" gateway_core yaml-cpp)

  # Integration test: full gateway pipeline with synthetic data
  ament_add_gtest(test_gateway_pipeline
    test/test_gateway_pipeline.cpp
  )
  target_include_directories(test_gateway_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_gateway_pipeline ${NODE_DEPS})
  target_link_libraries(test_gateway_pipeline "${cpp_typesupport_target}" gateway_core yaml-cpp)
endif()

ament_package()
