cmake_minimum_required(VERSION 3.5)

set(project nmea0183)

project(libnmea0183 VERSION 1.0)

set(CMAKE_VERBOSE_MAKEFILE true)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


file(GLOB PROTO_SRCS_PATH src/*.cpp)

include_directories(
    include
    src
)

add_library(${project} SHARED
  ${PROTO_SRCS_PATH}
)

# namespaced alias
# add_library(${project}::${component} ALIAS ${component})

target_include_directories(${project}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

include(CMakePackageConfigHelpers)
include(CMakeDependentOption)
include(CheckCXXCompilerFlag)
include(GNUInstallDirs)

set_target_properties(${project} PROPERTIES
  VERSION "${PROJECT_VERSION}"
  SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
  PROJECT_LABEL "${PROJECT_NAME} ${${PROJECT_NAME}-label-postfix}"
  DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")

# FIXME(felix2012): A more common place for the cmake export would be
# `CMAKE_INSTALL_LIBDIR`, as e.g. done in ubuntu or in this project for GTest
set(CONFIG_EXPORT_DIR "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}")
set(EXPORT_TARGETS ${PROJECT_NAME})

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  INSTALL_DESTINATION "${CONFIG_EXPORT_DIR}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CONFIG_EXPORT_DIR)
unset(EXPORT_TARGETS)

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  COMPATIBILITY AnyNewerVersion)

# configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)

install(TARGETS ${project}
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
              FILES_MATCHING PATTERN "*.h")
install(EXPORT ${PROJECT_NAME}-targets
  DESTINATION "${CONFIG_EXPORT_DIR}")
install(FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION "${CONFIG_EXPORT_DIR}")
# install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc"
  # DESTINATION ${CMAKE_INSTALL_DATADIR}/pkgconfig)

# build unit test
add_executable(test examples/test.cpp)
target_link_libraries(test nmea0183)

# install(TARGETS test DESTINATION lib/test)
