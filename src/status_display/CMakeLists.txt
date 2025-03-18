cmake_minimum_required(VERSION 3.8)
project(status_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(status_interfaces REQUIRED)
#REQUIRED：表示如果找不到Qt5库或者指定的组件，CMake会报错并终止配置过程。这确保了项目依赖的库必须存在。
#COMPONENTS Widgets：指定需要Qt5的Widgets模块。Qt是一个模块化的框架，Widgets模块提供了用于创建传统窗口应用程序的组件，例如窗口、按钮、菜单等
find_package(Qt5 REQUIRED COMPONENTS Widgets)   
add_executable(hello_qt src/hello_qt.cpp)
add_executable(system_display src/system_display.cpp)
#将Qt5::Widgets模块链接到hello_qt目标
target_link_libraries(hello_qt Qt5::Widgets)
target_link_libraries(system_display Qt5::Widgets)
ament_target_dependencies(system_display rclcpp status_interfaces)
install(TARGETS hello_qt system_display
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
