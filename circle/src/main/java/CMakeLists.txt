# -*- mode: cmake -*-
# vi: set ft=cmake :

#
# circle/src/main/java/CMakeLists.txt
# RVO2 Library Java
#
# Copyright 2008 University of North Carolina at Chapel Hill
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

if(BUILD_TESTING)
  get_target_property(RVOJava_RVO_JAR_FILE RVO JAR_FILE)
  get_filename_component(RVOJava_RVO_JAR_FILE_NAME "${RVOJava_RVO_JAR_FILE}"
    NAME)

  configure_file(Circle.MANIFEST.MF.in Circle.MANIFEST.MF)

  add_jar(Circle
    SOURCES ${RVOJava_JAR_CLASSES_PREFIX}/examples/Circle.java
    INCLUDE_JARS RVO
    ENTRY_POINT ${RVOJava_PACKAGE_NAME}.examples.Circle
    OUTPUT_NAME circle
    MANIFEST "${CMAKE_CURRENT_BINARY_DIR}/Circle.MANIFEST.MF")

  get_target_property(RVOJava_CIRCLE_JAR_FILE Circle JAR_FILE)

  add_test(NAME Circle
    COMMAND "${Java_JAVA_EXECUTABLE}" -jar "${RVOJava_CIRCLE_JAR_FILE}")
  set_tests_properties(Circle PROPERTIES
    LABELS medium
    TIMEOUT 60)
endif()
