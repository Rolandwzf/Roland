cmake_minimum_required(VERSION 3.8)
project(status_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#rosidl_generate_interfaces() 是 ROS 2 的 CMake 指令，用于生成 C++、Python 等语言的 消息（msg）、服务（srv）或动作（action） 接口代码
#${PROJECT_NAME} 代表当前 CMake 项目的名称，即 status_interfaces
#msg/SystemStatus.msg 是消息接口定义文件的路径
#DEPENDENCIES builtin_interfaces 表示 SystemStatus.msg 文件中使用了 builtin_interfaces 包中的数据类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemStatus.msg"
  DEPENDENCIES builtin_interfaces

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
