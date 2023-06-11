# -*- mode: bazel; -*-
# vi: set ft=bazel:

#
# extensions.bzl
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

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _non_module_deps_implementation(ctx):
    http_archive(
        name = "google_bazel_common",
        sha256 = "17ea98149586dff60aa741c67fbd9a010fbb1507df90e741c50403bf5228bea3",
        strip_prefix = "bazel-common-a9e1d8efd54cbf27249695b23775b75ca65bb59d",
        urls = ["https://github.com/google/bazel-common/archive/a9e1d8efd54cbf27249695b23775b75ca65bb59d.tar.gz"],
    )

non_module_deps = module_extension(
    implementation = _non_module_deps_implementation,
)
