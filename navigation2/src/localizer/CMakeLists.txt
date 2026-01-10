cmake_minimum_required(VERSION 3.8)
project(localizer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interface REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # 关键！

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


set(SRC_LIST src/localizers/commons.cpp
             src/localizers/icp_localizer.cpp)

add_executable(localizer_node src/localizer_node.cpp ${SRC_LIST})
ament_target_dependencies(localizer_node rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs interface)
target_link_libraries(localizer_node ${PCL_LIBRARIES}  yaml-cpp) 

# CMakeLists.txt 增加
add_executable(frame_projector_node src/frame_projector_node.cpp)
ament_target_dependencies(frame_projector_node rclcpp tf2_ros tf2 geometry_msgs tf2_geometry_msgs)

# 强制使用 C++ 头文件（Humble 必须！）
target_compile_definitions(frame_projector_node PUBLIC TF2_CPP_HEADERS)

install(TARGETS
  frame_projector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS localizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# ============ 安装 vel 节点 ============
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)
# 在 localizer/CMakeLists.txt 最后加入
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
