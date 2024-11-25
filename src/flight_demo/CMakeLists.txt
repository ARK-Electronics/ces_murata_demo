cmake_minimum_required(VERSION 3.8)
project(flight_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(flight_demo 
  FlightDemo.cpp)

  ament_target_dependencies(flight_demo rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)

  target_compile_features(flight_demo PUBLIC c_std_99 cxx_std_17)
  
  install(TARGETS flight_demo DESTINATION lib/${PROJECT_NAME})
  
  if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
  endif()
  
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
  
  install(DIRECTORY cfg
    DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
