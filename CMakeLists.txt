cmake_minimum_required(VERSION 2.8.3)
project(osx_joystick)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_boost_directories()



find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
# add_message_files(
#   DIRECTORY msg
#   FILES
#   Message1.msg
#   Message2.msg
# )


################ GLFW ##############################
find_package(PkgConfig)
pkg_check_modules(GLFW REQUIRED libglfw)
if(GLFW_FOUND)
	message("======== FOUND GLFW ${GLFW_VERSION} =============")
	include_directories(${GLFW_INCLUDE_DIRS})
endif(GLFW_FOUND)

################ JoyStick ##############################
add_executable(joystick src/joystick.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GLFW_LIBRARIES})



################ TwistJoyStick ########################
#rosbuild_add_executable(twist_js src/twist_js.cpp)
#rosbuild_link_boost(twist_js program_options)
#set(CMAKE_EXE_LINKER_FLAGS "-framework Cocoa -framework OpenGL -framework IOKit -lglfw" )

add_executable(twist_js src/twist_js.cpp)
target_link_libraries(twist_js ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GLFW_LIBRARIES})


message("=  ${CATKIN_PACKAGE_BIN_DESTINATION} =")
install(TARGETS joystick RUNTIME DESTINATION bin)
