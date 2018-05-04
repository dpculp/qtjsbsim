cmake_minimum_required(VERSION 2.8.3)
set( cmake_verbose_makefile TRUE)

PROJECT(qtjsbsim)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

FIND_PACKAGE(Qt5 COMPONENTS Core Gui Network Svg WebKit WebKitWidgets)

if(WIN32)
  set (QT_USE_QTMAIN TRUE)
endif(WIN32)

set(ENABLE_PLIB_JOYSTICK 1)
find_package(PLIB REQUIRED js)

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}"
  "${CMAKE_SOURCE_DIR}/src"
  "${PROJECT_BINARY_DIR}/src"
  "${CMAKE_SOURCE_DIR}/src/QFI"
  "${CMAKE_SOURCE_DIR}/src/autopilot"
  "${CMAKE_SOURCE_DIR}/src/nav"
  "${CMAKE_SOURCE_DIR}/src/viewer"
  "${CMAKE_SOURCE_DIR}/src/debug"
  "${CMAKE_SOURCE_DIR}/src/jsbsim"
  "${CMAKE_SOURCE_DIR}/src/joystick"
  "${CMAKE_SOURCE_DIR}/src/flightgear"
  ${PLIB_INCLUDE_DIR}
  )

ADD_SUBDIRECTORY(src)

