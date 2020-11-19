cmake_minimum_required(VERSION 2.8.3)
project(ecam_v4l2)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG")
set(ROSLINT_CPP_OPTS "--filter=+,-build/c++11")
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  sensor_msgs
  std_msgs
)
add_definitions(-std=c++11 -Wall -g)
find_package(PkgConfig REQUIRED)
pkg_check_modules (libudev REQUIRED libudev)


add_service_files(FILES
camera.srv
query_control.srv
set_control.srv
set_format.srv
enum_format.srv)

add_message_files(FILES image.msg)

generate_messages(DEPENDENCIES std_msgs)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
INCLUDE_DIRS include
LIBRARIES ecam_v4l2
CATKIN_DEPENDS std_msgs roscpp message_runtime
)
add_library(ecam_v4l2
  src/camera.cpp
  src/device.cpp
  src/srv_servers.cpp
  )
## Declare a C++ library
add_executable(ecam_v4l2_node
		src/ecam_v4l2.cpp
 )
target_link_libraries(${PROJECT_NAME}_node
   ecam_v4l2
   ${catkin_LIBRARIES}
   ${libudev_LIBRARIES}
 )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
