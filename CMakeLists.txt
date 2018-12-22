cmake_minimum_required(VERSION 2.8.3)

project(rqt_mcbtest)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  medlab_motor_control_board
)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

# include(${qt5_use_modules})

find_path(
  QWT_INCLUDE_DIRS
  NAMES
    qwt_counter.h
  PATHS
    /usr/local/qwt-6.1.3/include
)

find_library(
  QWT_LIBRARIES
  NAMES
    qwt-qt5
)

set(rqt_mcbtest_SRCS
  src/mcbtest.cpp
#  src/mcbros.cpp
  src/gainsdialog.cpp
)

set(rqt_mcbtest_HDRS
  include/mcbtest.h
#  include/mcbros.h
  include/gainsdialog.h
)

set(rqt_mcbtest_UIS
  src/mcb_gui.ui
  src/gainsdialog.ui
)

set(rqt_mcbtest_QRCS
  resource/mcb_gui_qrc.qrc
)

set(rqt_mcbtest_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
#  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${rqt_mcbtest_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp medlab_motor_control_board
)

qt5_wrap_cpp(rqt_mcbtest_MOCS ${rqt_mcbtest_HDRS})
qt5_add_resources(QT_RESOURCES_CPP ${rqt_mcbtest_QRCS})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
qt5_wrap_ui(rqt_mcbtest_UIS_H ${rqt_mcbtest_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
  ${rqt_mcbtest_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${QWT_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  ${rqt_mcbtest_SRCS}
  ${rqt_mcbtest_MOCS}
  ${rqt_mcbtest_UIS_H}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} ${QWT_LIBRARIES})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.png"
)
