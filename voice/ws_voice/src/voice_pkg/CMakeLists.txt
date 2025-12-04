cmake_minimum_required(VERSION 3.8)
project(voice_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(talker src/talker.cpp)
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(talker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  talker
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

# 添加语音LLM节点可执行文件
add_executable(voice_llm_node src/voice_llm_node.cpp)
target_include_directories(voice_llm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /home/tang/voice/SparkChain_Linux_SDK_2.0.0_rc1/SparkChain_Linux_SDK_2.0.0_rc1/include)
target_compile_features(voice_llm_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  voice_llm_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

# 链接科大讯飞SDK库
target_link_libraries(voice_llm_node
  /home/tang/voice/SparkChain_Linux_SDK_2.0.0_rc1/SparkChain_Linux_SDK_2.0.0_rc1/libs/libSparkChain.so
)

# 设置RPATH，使可执行文件能找到共享库
set_target_properties(voice_llm_node PROPERTIES
  INSTALL_RPATH "/home/tang/voice/SparkChain_Linux_SDK_2.0.0_rc1/SparkChain_Linux_SDK_2.0.0_rc1/libs;$ORIGIN"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# 安装科大讯飞SDK库到install目录
install(FILES
  /home/tang/voice/SparkChain_Linux_SDK_2.0.0_rc1/SparkChain_Linux_SDK_2.0.0_rc1/libs/libSparkChain.so
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  talker
  voice_llm_node
  DESTINATION lib/${PROJECT_NAME})

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
