# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# CMakeLists.txt
# RVO2 Library Java
#
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Please send all bug reports to <geom@cs.unc.edu>.
#
# The authors may be contacted via:
#
# Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
# Dept. of Computer Science
# 201 S. Columbia St.
# Frederick P. Brooks, Jr. Computer Science Bldg.
# Chapel Hill, N.C. 27599-3175
# United States of America
#
# <https://gamma.cs.unc.edu/RVO2/>
#

cmake_minimum_required(VERSION 3.26)

project(RVOJava
  VERSION 2.0.0
  DESCRIPTION "Optimal Reciprocal Collision Avoidance for Java"
  HOMEPAGE_URL https://gamma.cs.unc.edu/RVO2/
  LANGUAGES NONE)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # cmake-lint: disable=C0301
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build; options are Debug Release RelWithDebInfo MinSizeRel None"
    FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS Debug Release RelWithDebInfo MinSizeRel None)
endif()

include(CTest)

find_package(Java 21 REQUIRED COMPONENTS Development)
include(UseJava)

option(WARNINGS_AS_ERRORS "Turn compiler warnings into errors" OFF)

if(WARNINGS_AS_ERRORS)
  list(APPEND CMAKE_JAVA_COMPILE_FLAGS -Werror)
endif()

set(CMAKE_JAVA_TARGET_OUTPUT_DIR "${PROJECT_BINARY_DIR}/java")
file(MAKE_DIRECTORY "${CMAKE_JAVA_TARGET_OUTPUT_DIR}")

include(FetchContent)

# cmake-lint: disable=C0301
FetchContent_Declare(org_apache_commons_commons_math3
  DOWNLOAD_NO_EXTRACT ON
  URL https://repo1.maven.org/maven2/org/apache/commons/commons-math3/3.6.1/commons-math3-3.6.1.jar
  URL_HASH SHA256=1e56d7b058d28b65abd256b8458e3885b674c1d588fa43cd7d1cbb9c7ef2b308
)

FetchContent_MakeAvailable(org_apache_commons_commons_math3)

find_jar(RVOJava_COMMONS_MATH3_JAR_FILE
  NAMES commons-math3
  PATHS "${org_apache_commons_commons_math3_SOURCE_DIR}"
  VERSIONS 3.6.1)

if(RVOJava_COMMONS_MATH3_JAR_FILE)
  message(STATUS
    "Found commons-math3 JAR file: ${RVOJava_COMMONS_MATH3_JAR_FILE}")
else()
  message(FATAL_ERROR "Could NOT find commons-math3 JAR file")
endif()

configure_file("${RVOJava_COMMONS_MATH3_JAR_FILE}"
  "${CMAKE_JAVA_TARGET_OUTPUT_DIR}" COPYONLY)

get_filename_component(RVOJava_COMMONS_MATH3_JAR_FILE_NAME
  "${RVOJava_COMMONS_MATH3_JAR_FILE}" NAME)

list(APPEND CMAKE_JAVA_INCLUDE_PATH
  "${CMAKE_JAVA_TARGET_OUTPUT_DIR}/${RVOJava_COMMONS_MATH3_JAR_FILE_NAME}")

set(RVOJava_DIR share/cmake/${PROJECT_NAME})
set(RVOJava_DOC_DIR share/doc/${PROJECT_NAME})

set(RVOJava_JAR_CLASSES_PREFIX edu/unc/cs/gamma/rvo)
string(REPLACE "/" "." RVOJava_PACKAGE_NAME ${RVOJava_JAR_CLASSES_PREFIX})
set(RVOJava_PACKAGE_VENDOR "University of North Carolina at Chapel Hill")

add_subdirectory(rvo/src/main/java)
add_subdirectory(blocks/src/main/java)
add_subdirectory(circle/src/main/java)

install(FILES LICENSE
  COMPONENT runtime
  DESTINATION ${RVOJava_DOC_DIR})

install(FILES "${RVOJava_COMMONS_MATH3_JAR_FILE}"
  COMPONENT runtime
  DESTINATION share/java)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${RVOJava_DIR})

write_basic_package_version_file(${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  COMPONENT development
  DESTINATION ${RVOJava_DIR})

include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_CONTACT "Jamie Snape")
set(CPACK_PACKAGE_VENDOR "${RVOJava_PACKAGE_VENDOR}")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_CHECKSUM SHA256)
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
set(CPACK_STRIP_FILES ON)

set(CPACK_SOURCE_IGNORE_FILES
  _build/ \\\\.git/ \\\\.gitattributes \\\\.github/ \\\\.gitignore bazel-
  Brewfile)
set(CPACK_SOURCE_STRIP_FILES ON)

set(CPACK_DEBIAN_COMPRESSION_TYPE zstd)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE all)
set(CPACK_DEBIAN_PACKAGE_DEPENDS)
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE ${PROJECT_HOMEPAGE_URL})
set(CPACK_DEBIAN_PACKAGE_SECTION contrib/java)

set(CPACK_FREEBSD_PACKAGE_DEPS)
set(CPACK_FREEBSD_PACKAGE_LICENSE APACHE20)
set(CPACK_FREEBSD_PACKAGE_WWW ${PROJECT_HOMEPAGE_URL})

set(CPACK_RPM_FILE_NAME RPM-DEFAULT)
set(CPACK_RPM_PACKAGE_ARCHITECTURE noarch)
set(CPACK_RPM_PACKAGE_LICENSE "ASL 2.0")
set(CPACK_RPM_PACKAGE_REQUIRES "java-headless >= 1:1.8")
set(CPACK_RPM_PACKAGE_URL ${PROJECT_HOMEPAGE_URL})

include(CPack)
