cmake_minimum_required(VERSION 3.8)
project(onrobot2_rg_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS  
    scripts/OnRobotRGTcpNode.py
    scripts/OnRobotRGStatusListener.py
    scripts/OnRobotRGSimpleControllerServer.py 
    scripts/OnRobotRGSimpleController.py 
    scripts/DualChanger/OnRobotRGTcpDualNode.py 
    scripts/DualChanger/OnRobotRGStatusDualListener.py 
    scripts/DualChanger/OnRobotRGDualServer.py 
    DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
