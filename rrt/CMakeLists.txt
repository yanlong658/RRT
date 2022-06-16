cmake_minimum_required(VERSION 3.5)
project(rrt)


find_package(ament_cmake REQUIRED )
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  Eigen3
  OpenCV
)

include_directories(
 include
 ${EIGEN3_INCLUDE_DIR}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(main
  src/main.cpp
)
ament_target_dependencies(main ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS
  main
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
