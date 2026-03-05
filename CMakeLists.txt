cmake_minimum_required(VERSION 3.8)
project(poise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ── Generate custom messages ─────────────────────────────────────────────────
# rosidl installs the poise.msg Python module automatically.
# Do NOT call ament_python_install_package(poise) — it would conflict.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IntegrityStatus.msg"
  DEPENDENCIES std_msgs
)

# ── Install poise/qos.py alongside the rosidl-generated Python package ───────
# This makes `from poise.qos import ...` work in all installed executables.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES
  poise/qos.py
  DESTINATION ${PYTHON_INSTALL_DIR}/poise/
)

# ── Install Python executables ────────────────────────────────────────────────
install(PROGRAMS
  poise/sim/gnss_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gnss_publisher
)

install(PROGRAMS
  poise/sim/imu_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_publisher
)

install(PROGRAMS
  poise/checks/gnss_imu_checker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gnss_imu_checker
)

install(PROGRAMS
  poise/core/integrity_aggregator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME integrity_aggregator
)

# ── Install config, launch, and docs ─────────────────────────────────────────
install(DIRECTORY
  config
  launch
  docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime std_srvs)
ament_package()
