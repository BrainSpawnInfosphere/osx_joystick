cmake_minimum_required(VERSION 2.8.3)
project(osx_joystick)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(PkgConfig)
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


#message("=  ${CATKIN_PACKAGE_BIN_DESTINATION} =")
install(TARGETS joystick twist_js RUNTIME DESTINATION bin)
