cmake_minimum_required(VERSION 2.8.3)
project(osx_joystick)

set(CMAKE_EXE_LINKER_FLAGS "-framework Cocoa -framework OpenGL -framework IOKit")

find_package(catkin REQUIRED roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW libglfw)

catkin_package()

include_directories(${GLFW_INCLUDE_DIRS})

add_executable(joystick src/joystick.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})

add_executable(twist_js src/twist_js.cpp)
target_link_libraries(twist_js ${catkin_LIBRARIES} ${GLFW_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS joystick twist_js
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
