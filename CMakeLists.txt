cmake_minimum_required(VERSION 2.8.3)
project(kdl_wrapper)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp urdf kdl_acc_solver)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS urdf kdl_acc_solver
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
)

add_library(${PROJECT_NAME} src/kdl_wrapper.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_executable(pr2_kdl_wrapper_example src/pr2_kdl_wrapper_example.cpp)
target_link_libraries(pr2_kdl_wrapper_example kdl_wrapper ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS ${PROJECT_NAME} pr2_kdl_wrapper_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
