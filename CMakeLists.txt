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

# ── Codegen: sensors.yaml → generated/parsed_{transducers,gateway,proxies}.hpp ──
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GEN_DIR  ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GEN_YAML ${CMAKE_CURRENT_SOURCE_DIR}/config/sensors.yaml)
set(GEN_TOOL ${CMAKE_CURRENT_SOURCE_DIR}/codegen/generate.py)

set(GEN_OUTS
  ${GEN_DIR}/parsed_transducers.hpp
  ${GEN_DIR}/parsed_gateway.hpp
  ${GEN_DIR}/parsed_proxies.hpp
)

add_custom_command(
  OUTPUT  ${GEN_OUTS}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_DIR}
  COMMAND ${Python3_EXECUTABLE} ${GEN_TOOL}
          --yaml ${GEN_YAML} --out ${GEN_DIR}
  DEPENDS ${GEN_YAML} ${GEN_TOOL}
  COMMENT "Generating SmartData transducers from ${GEN_YAML}"
)
add_custom_target(gateway_codegen DEPENDS ${GEN_OUTS})

# ── smartdata_runtime: SmartData run-time (network, scheduler) ────────────────
find_package(Threads REQUIRED)
add_library(smartdata_runtime SHARED src/smartdata/network.cpp)
target_include_directories(smartdata_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(smartdata_runtime Threads::Threads)

# ── gateway_node ──────────────────────────────────────────────────────────────
add_executable(gateway_node src/gateway_node.cpp)
add_dependencies(gateway_node gateway_codegen)
target_include_directories(gateway_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${GEN_DIR}>
)
ament_target_dependencies(gateway_node ${NODE_DEPS})
target_link_libraries(gateway_node "${cpp_typesupport_target}" smartdata_runtime)

# ── proxy_node ────────────────────────────────────────────────────────────
add_executable(proxy_node src/proxy_node.cpp)
add_dependencies(proxy_node gateway_codegen)
target_include_directories(proxy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${GEN_DIR}>
)
ament_target_dependencies(proxy_node ${NODE_DEPS})
target_link_libraries(proxy_node "${cpp_typesupport_target}" smartdata_runtime)

# ── demo_publisher_node ───────────────────────────────────────────────────────
add_executable(demo_publisher_node src/demo_publisher_node.cpp)
target_include_directories(demo_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(demo_publisher_node ${NODE_DEPS})
target_link_libraries(demo_publisher_node "${cpp_typesupport_target}")

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS gateway_node proxy_node demo_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS smartdata_runtime
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

  # 32-bit dimensional SI unit encoding (Fig. 1 of the paper)
  ament_add_gtest(test_si_units test/test_si_units.cpp)
  target_include_directories(test_si_units PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # SmartData dynamic: interests, responses, periods, regions, commands
  ament_add_gtest(test_smartdata_runtime test/test_smartdata_runtime.cpp TIMEOUT 120)
  target_link_libraries(test_smartdata_runtime smartdata_runtime)

  # Generated transducers + translation latency / throughput
  ament_add_gtest(test_gateway_pipeline test/test_gateway_pipeline.cpp TIMEOUT 120)
  add_dependencies(test_gateway_pipeline gateway_codegen)
  target_include_directories(test_gateway_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GEN_DIR}>
  )
  ament_target_dependencies(test_gateway_pipeline ${NODE_DEPS})
  target_link_libraries(test_gateway_pipeline "${cpp_typesupport_target}" smartdata_runtime)

  # End to end: ROS → gateway → SmartData network → proxies → ROS (and commands back)
  ament_add_gtest(test_bus_roundtrip test/test_bus_roundtrip.cpp TIMEOUT 120)
  add_dependencies(test_bus_roundtrip gateway_codegen)
  target_include_directories(test_bus_roundtrip PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GEN_DIR}>
  )
  ament_target_dependencies(test_bus_roundtrip ${NODE_DEPS})
  target_link_libraries(test_bus_roundtrip "${cpp_typesupport_target}" smartdata_runtime)

  # Imports: SmartData of other nodes → proxy → ROS (own YAML, own headers)
  set(GEN_TEST_DIR  ${CMAKE_CURRENT_BINARY_DIR}/generated_imports_test)
  set(GEN_TEST_YAML ${CMAKE_CURRENT_SOURCE_DIR}/test/imports_test.yaml)
  set(GEN_TEST_OUTS
    ${GEN_TEST_DIR}/parsed_transducers.hpp
    ${GEN_TEST_DIR}/parsed_gateway.hpp
    ${GEN_TEST_DIR}/parsed_proxies.hpp
  )
  add_custom_command(
    OUTPUT  ${GEN_TEST_OUTS}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_TEST_DIR}
    COMMAND ${Python3_EXECUTABLE} ${GEN_TOOL}
            --yaml ${GEN_TEST_YAML} --out ${GEN_TEST_DIR}
    DEPENDS ${GEN_TEST_YAML} ${GEN_TOOL}
    COMMENT "Generating SmartData transducers from ${GEN_TEST_YAML}"
  )
  add_custom_target(imports_test_codegen DEPENDS ${GEN_TEST_OUTS})

  ament_add_gtest(test_imports test/test_imports.cpp TIMEOUT 120)
  add_dependencies(test_imports imports_test_codegen)
  target_include_directories(test_imports PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GEN_TEST_DIR}>
  )
  ament_target_dependencies(test_imports ${NODE_DEPS})
  target_link_libraries(test_imports "${cpp_typesupport_target}" smartdata_runtime)
endif()

ament_package()
