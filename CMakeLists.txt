cmake_minimum_required(VERSION 2.8.3)
project(kdl_wrapper)

find_package(catkin REQUIRED COMPONENTS roscpp orocos_kdl urdf kdl_acc_solver)

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS orocos_kdl urdf kdl_acc_solver
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/kdl_wrapper.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser)

add_executable(pr2_kdl_wrapper_example src/pr2_kdl_wrapper_example.cpp)
target_link_libraries(pr2_kdl_wrapper_example kdl_wrapper ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} pr2_kdl_wrapper_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
