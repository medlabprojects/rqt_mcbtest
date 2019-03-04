cmake_minimum_required(VERSION 3.0)

project(rqt_mcbtest)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  medlab_motor_control_board
)

# ensure we use correct Qt version
set(QT_INSTALL_PATH /opt/Qt)
file( GLOB_RECURSE sres ${QT_INSTALL_PATH}/*/Qt5Config.cmake )
get_filename_component( Qt5_DIR ${sres} DIRECTORY )
find_package(
  Qt5 5.11
  COMPONENTS
    Core
    Widgets
  REQUIRED
)
#find_package(Qt5Core REQUIRED)
#find_package(Qt5Widgets REQUIRED)

find_path(
  QWT_INCLUDE_DIRS
  NAMES
    qwt_counter.h
  PATHS
    /usr/local/qwt-6.1.4-qt-5.5.1/include
)

find_library(
  QWT_LIBRARIES
  NAMES
    qwt
  PATHS
    /usr/local/qwt-6.1.4-qt-5.5.1/lib
)

set(rqt_mcbtest_SRCS
  src/mcbtest.cpp
  src/gainsdialog.cpp
  src/gainsdialog.ui
  src/mcbtest_gui.ui
)

# NOTE: only need to list headers that require moc'ing (i.e. those that inherit QObject)
set(${PROJECT_NAME}_MOCS
  include/${PROJECT_NAME}/mcbtest.h
  include/${PROJECT_NAME}/gainsdialog.h
)

set(${PROJECT_NAME}_UIS
  src/mcbtest_gui.ui
  src/gainsdialog.ui
)

#set(${PROJECT_NAME}_QRCS
#  resource/mcb_gui_qrc.qrc
#)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
)

catkin_package(
  INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rqt_gui
    rqt_gui_cpp
    medlab_motor_control_board
)

qt5_wrap_cpp(${PROJECT_NAME}_MOCS_H ${${PROJECT_NAME}_MOCS})
#qt5_add_resources(QT_RESOURCES_CPP ${${PROJECT_NAME}_QRCS})

# ensure generated header files are being created in the devel space
#set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
#set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
#set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  "${CMAKE_CURRENT_BINARY_DIR}"
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${QWT_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${rqt_mcbtest_SRCS}
  ${rqt_mcbtest_MOCS_H}
  ${rqt_mcbtest_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  ${QWT_LIBRARIES}
)


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

#install(
#  DIRECTORY resource
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  FILES_MATCHING
#  PATTERN "*.png"
#)
