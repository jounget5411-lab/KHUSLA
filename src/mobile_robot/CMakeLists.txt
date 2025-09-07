cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch model parameters worlds scripts   # ← worlds 추가
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 실행형 파이썬 노드 설치 (lib/<pkg> 밑으로)
install(
  PROGRAMS
    scripts/vehicle_interface.py
    scripts/steering_pot_emulator.py
    scripts/twist_to_ackermann_fixed.py
    scripts/posev_to_model_pose.py      # 이 줄 추가
    scripts/worldpose_to_navsat.py    # 이 줄 추가
    scripts/imu_cov_injector.py        # ★ 추가
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
