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
find_package(rcpputils REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Message generation ────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SmartData.msg"
  DEPENDENCIES std_msgs
)

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
  rcpputils
  ament_index_cpp
)

# ── Codegen: sensors.yaml → generated/parsed_{decompose,dispatch,recompose,sd_dispatch}.hpp ──
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GEN_DIR  ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GEN_YAML ${CMAKE_CURRENT_SOURCE_DIR}/config/sensors.yaml)
set(GEN_TOOL ${CMAKE_CURRENT_SOURCE_DIR}/codegen/generate.py)

set(GEN_OUTS
  ${GEN_DIR}/parsed_decompose.hpp
  ${GEN_DIR}/parsed_dispatch.hpp
  ${GEN_DIR}/parsed_recompose.hpp
  ${GEN_DIR}/parsed_sd_dispatch.hpp
)

add_custom_command(
  OUTPUT  ${GEN_OUTS}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_DIR}
  COMMAND ${Python3_EXECUTABLE} ${GEN_TOOL}
          --yaml ${GEN_YAML} --out ${GEN_DIR}
  DEPENDS ${GEN_YAML} ${GEN_TOOL}
  COMMENT "Generating per-sensor codegen from ${GEN_YAML}"
)
add_custom_target(gateway_codegen DEPENDS ${GEN_OUTS})

# ── sd_bus library: UDP multicast SmartData wire bus ──────────────────────────
add_library(sd_bus SHARED src/sd_bus.cpp)
target_include_directories(sd_bus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(sd_bus "${cpp_typesupport_target}")

# ── gateway_node ──────────────────────────────────────────────────────────────
add_executable(gateway_node src/gateway_node.cpp)
add_dependencies(gateway_node gateway_codegen)
target_include_directories(gateway_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${GEN_DIR}>
)
ament_target_dependencies(gateway_node ${NODE_DEPS})
target_link_libraries(gateway_node "${cpp_typesupport_target}" sd_bus)

# ── sd_to_ros_node ────────────────────────────────────────────────────────────
add_executable(sd_to_ros_node src/sd_to_ros_node.cpp)
add_dependencies(sd_to_ros_node gateway_codegen)
target_include_directories(sd_to_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${GEN_DIR}>
)
ament_target_dependencies(sd_to_ros_node ${NODE_DEPS})
target_link_libraries(sd_to_ros_node "${cpp_typesupport_target}" sd_bus)

# ── demo_publisher_node ───────────────────────────────────────────────────────
add_executable(demo_publisher_node src/demo_publisher_node.cpp)
target_include_directories(demo_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(demo_publisher_node ${NODE_DEPS})
target_link_libraries(demo_publisher_node "${cpp_typesupport_target}")

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS gateway_node sd_to_ros_node demo_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS sd_bus
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

  # Unit tests: 32-bit dimensional SI unit encoding (Fig. 1 of the paper)
  ament_add_gtest(test_si_units
    test/test_si_units.cpp
  )
  target_include_directories(test_si_units PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # Integration test: full gateway pipeline with synthetic data (codegen path)
  ament_add_gtest(test_gateway_pipeline
    test/test_gateway_pipeline.cpp
  )
  add_dependencies(test_gateway_pipeline gateway_codegen)
  target_include_directories(test_gateway_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GEN_DIR}>
  )
  ament_target_dependencies(test_gateway_pipeline ${NODE_DEPS})
  target_link_libraries(test_gateway_pipeline "${cpp_typesupport_target}")

  # Fidelity test: ROS → decompose_<name> → feed_<name> → ROS (codegen pair)
  ament_add_gtest(test_validator
    test/test_validator.cpp
  )
  add_dependencies(test_validator gateway_codegen)
  target_include_directories(test_validator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GEN_DIR}>
  )
  ament_target_dependencies(test_validator ${NODE_DEPS})
  target_link_libraries(test_validator "${cpp_typesupport_target}")

  # Synthetic recompose: hand-built SmartData → dispatch_sd → reconstructed ROS
  ament_add_gtest(test_synthetic_recompose
    test/test_synthetic_recompose.cpp
  )
  add_dependencies(test_synthetic_recompose gateway_codegen)
  target_include_directories(test_synthetic_recompose PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GEN_DIR}>
  )
  ament_target_dependencies(test_synthetic_recompose ${NODE_DEPS})
  target_link_libraries(test_synthetic_recompose "${cpp_typesupport_target}")

  # End-to-end bus roundtrip: ROS → gateway → UDP multicast → sd_to_ros → ROS
  ament_add_gtest(test_bus_roundtrip
    test/test_bus_roundtrip.cpp
  )
  add_dependencies(test_bus_roundtrip gateway_codegen)
  target_include_directories(test_bus_roundtrip PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GEN_DIR}>
  )
  ament_target_dependencies(test_bus_roundtrip ${NODE_DEPS})
  target_link_libraries(test_bus_roundtrip "${cpp_typesupport_target}" sd_bus)
endif()

ament_package()
