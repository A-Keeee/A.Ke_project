cmake_minimum_required(VERSION 3.8)
project(armor_pnp_learn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)  # 用于坐标变换
find_package(tf2_ros REQUIRED)  # 用于在ROS2中使用tf2

# # 包含头文件路径
# include_directories(
#   include
#   ${OpenCV_INCLUDE_DIRS}
#   ${tf2_INCLUDE_DIRS}
#   ${tf2_ros_INCLUDE_DIRS}
# )

add_executable(armor_pnp_learn src/armor_pnp_learn.cpp src/inference.cpp)

ament_target_dependencies(armor_pnp_learn
  rclcpp
  tf2
  tf2_ros
  OpenCV
)

# target_link_libraries(armor_pnp_learn ${OpenCV_LIBRARIES} 
# tf2
# tf2_ros)

install(TARGETS
  armor_pnp_learn
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
