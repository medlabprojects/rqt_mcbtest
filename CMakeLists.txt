cmake_minimum_required(VERSION 3.1)
project(rqt_mcbtest)

set(CMAKE_CXX_STANDARD 14)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    medlab_motor_control_board
)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
#find_package(
#  Qt5
#  COMPONENTS
#    Core
#    Widgets
#  REQUIRED
#)

# NOTE: may need to update paths depending on where you installed Qwt
set(QWT_DIR
  /usr/local/qwt-6.1.4-qt-5.5.1
)

find_path(
  QWT_INCLUDE_DIRS
  NAMES
    qwt_counter.h
  PATHS
    ${QWT_DIR}/include
)
find_library(
  QWT_LIBRARIES
  NAMES
    qwt
  PATHS
    ${QWT_DIR}/lib
)

# source files
set(${PROJECT_NAME}_SRCS
  src/mcbtest.cpp
  src/gainsdialog.cpp
  src/gainsdialog.ui
  src/mcbtest_gui.ui
)

# location of header files
set(${PROJECT_NAME}_INCLUDES
  include
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


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS
    ${${PROJECT_NAME}_INCLUDES}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rqt_gui
    rqt_gui_cpp
    medlab_motor_control_board
)

###########
## Build ##
###########

# create MOC code for classes using QObject
qt5_wrap_cpp(${PROJECT_NAME}_MOCS_H ${${PROJECT_NAME}_MOCS})

# create headers from .ui files
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

#qt5_add_resources(QT_RESOURCES_CPP ${${PROJECT_NAME}_QRCS})

include_directories(
  ${${PROJECT_NAME}_INCLUDES}
  "${CMAKE_CURRENT_BINARY_DIR}"
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${QWT_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS_H}
  ${${PROJECT_NAME}_UIS_H}
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
