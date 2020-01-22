cmake_minimum_required(VERSION 2.8.3)

project(rqt_cam)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp message_generation cv_bridge)
find_package(OpenCV REQUIRED)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(rqt_cam_SRCS
  src/rqt_cam/image_view.cpp
  src/rqt_cam/ratio_layouted_frame.cpp
  src/rqt_cam/controls.cpp
  src/rqt_cam/ui.cpp
)

set(rqt_cam_HDRS
  include/rqt_cam/image_view.h
  include/rqt_cam/ratio_layouted_frame.h
  include/rqt_cam/controls.h
  include/rqt_cam/ui.h
)

set(rqt_cam_UIS
  src/rqt_cam/image_view.ui
)

set(rqt_cam_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_python_setup()

add_service_files(FILES
camera.srv
query_control.srv
set_control.srv
set_format.srv
enum_format.srv)

add_message_files(FILES image.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS ${rqt_cam_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp cv_bridge message_runtime
)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_cam_MOCS ${rqt_cam_HDRS})
else()
  qt4_wrap_cpp(rqt_cam_MOCS ${rqt_cam_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(rqt_cam_UIS_H ${rqt_cam_UIS})
else()
  qt4_wrap_ui(rqt_cam_UIS_H ${rqt_cam_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${rqt_cam_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_cam_SRCS} ${rqt_cam_MOCS} ${rqt_cam_UIS_H})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

