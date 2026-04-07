cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(LIBGPIOD_RaspberryPi_Library VERSION 2.0.0 LANGUAGES CXX)
include(GNUInstallDirs)

add_subdirectory(example)

add_compile_options(-Wall -Wconversion -Wextra -pedantic)

set(LIBSRC gpioevent.cpp)
set(LIBINCLUDE gpioevent.h)

add_library(gpioevent
  SHARED
  ${LIBSRC}
  )

set_target_properties(gpioevent PROPERTIES
  SOVERSION 2
  PUBLIC_HEADER "${LIBINCLUDE}")

target_link_libraries(gpioevent gpiodcxx)

install(TARGETS gpioevent EXPORT gpioevent-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})



add_library(gpioevent_static
  STATIC
  ${LIBSRC}
  )

set_target_properties(gpioevent_static PROPERTIES
  PUBLIC_HEADER "${LIBINCLUDE}")

target_link_libraries(gpioevent_static gpiodcxx)

install(TARGETS gpioevent_static EXPORT gpioevent_static-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
