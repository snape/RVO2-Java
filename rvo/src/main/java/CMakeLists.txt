# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# rvo/src/main/java/CMakeLists.txt
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

configure_file(RVO.MANIFEST.MF.in RVO.MANIFEST.MF)

add_jar(RVO
  SOURCES
    ${RVOJava_JAR_CLASSES_PREFIX}/Agent.java
    ${RVOJava_JAR_CLASSES_PREFIX}/KdTree.java
    ${RVOJava_JAR_CLASSES_PREFIX}/Line.java
    ${RVOJava_JAR_CLASSES_PREFIX}/Obstacle.java
    ${RVOJava_JAR_CLASSES_PREFIX}/package-info.java
    ${RVOJava_JAR_CLASSES_PREFIX}/RVOMath.java
    ${RVOJava_JAR_CLASSES_PREFIX}/Simulator.java
  MANIFEST "${CMAKE_CURRENT_BINARY_DIR}/RVO.MANIFEST.MF"
  VERSION ${PROJECT_VERSION}
  OUTPUT_NAME rvo)

export_jars(TARGETS RVO
  NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

if(WIN32)
  set_target_properties(RVO PROPERTIES
    INSTALL_DESTINATION share/java)
  get_target_property(RVOJava_RVO_JAR_FILE RVO JAR_FILE)
  install(FILES "${RVOJava_RVO_JAR_FILE}"
    DESTINATION share/java
    COMPONENT runtime)
else()
  install_jar(RVO
    DESTINATION share/java
    COMPONENT runtime)
endif()

install_jar_exports(TARGETS RVO
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${RVOJava_DIR}
  COMPONENT development)

option(BUILD_DOCUMENTATION "Build documentation" OFF)

if(BUILD_DOCUMENTATION)
  create_javadoc(api
    PACKAGES ${RVOJava_PACKAGE_NAME}
    SOURCEPATH "${CMAKE_CURRENT_SOURCE_DIR}"
    CLASSPATH "${CMAKE_JAVA_INCLUDE_PATH}"
    INSTALLPATH ${RVOJava_DOC_DIR}
    DOCTITLE "RVO2 Library Java"
    AUTHOR ON
    USE ON
    VERSION ON)
endif()


