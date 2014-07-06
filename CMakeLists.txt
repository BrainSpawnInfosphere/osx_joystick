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


## Declare things to be passed to other projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
  DEPENDS Boost # other-3rd-party-libraries
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


################ SDL2 ##############################
pkg_check_modules(SDL REQUIRED sdl2)
if(SDL_FOUND)
	message("======== FOUND SDL ${SDL_VERSION} =============")
	include_directories(${SDL_INCLUDE_DIRS})
endif(SDL_FOUND)

################ JoyStick ##############################
add_executable(joystick_window src/ps4.cpp)
target_link_libraries(joystick_window ${catkin_LIBRARIES} curses)

add_executable(${PROJECT_NAME} src/joystick.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SDL_LIBRARIES})



################ TwistJoyStick ########################
#rosbuild_add_executable(twist_js src/twist_js.cpp)
#rosbuild_link_boost(twist_js program_options)
#set(CMAKE_EXE_LINKER_FLAGS "-framework Cocoa -framework OpenGL -framework IOKit -lglfw" )

add_executable(osx_twist_js src/twist_js.cpp)
target_link_libraries(osx_twist_js ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SDL_LIBRARIES})


message("=  ${CATKIN_PACKAGE_BIN_DESTINATION} =")
install(TARGETS ${PROJECT_NAME} osx_twist_js RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS ${PROJECT_NAME} osx_twist_js RUNTIME DESTINATION "/Users/kevin/catkin/install")


## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )