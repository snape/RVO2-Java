<!--
QUALITY_DECLARATION.md
RVO2 Library Java

SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
SPDX-License-Identifier: CC-BY-SA-4.0

Creative Commons Attribution-ShareAlike 4.0 International Public License

You are free to:

* Share -- copy and redistribute the material in any medium or format

* ShareAlike -- If you remix, transform, or build upon the material, you must
  distribute your contributions under the same license as the original

* Adapt -- remix, transform, and build upon the material for any purpose, even
  commercially.

The licensor cannot revoke these freedoms as long as you follow the license
terms.

Under the following terms:

* Attribution -- You must give appropriate credit, provide a link to the
  license, and indicate if changes were made. You may do so in any reasonable
  manner, but not in any way that suggests the licensor endorses you or your
  use.

* No additional restrictions -- You may not apply legal terms or technological
  measures that legally restrict others from doing anything the license
  permits.

Notices:

* You do not have to comply with the license for elements of the material in
  the public domain or where your use is permitted by an applicable exception
  or limitation.

* No warranties are given. The license may not give you all of the permissions
  necessary for your intended use. For example, other rights such as publicity,
  privacy, or moral rights may limit how you use the material.

Please send all bug reports to <geom@cs.unc.edu>.

The authors may be contacted via:

Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
Dept. of Computer Science
201 S. Columbia St.
Frederick P. Brooks, Jr. Computer Science Bldg.
Chapel Hill, N.C. 27599-3175
United States of America

<https://gamma.cs.unc.edu/RVO2/>
-->

# `RVO2 Library Java` Quality Declaration

The package `RVO2 Library Java` claims to be in the **Quality Level 2** category.

Below are the rationales, notes, and caveats for this claim, organized by each
requirement listed in the Package Requirements for Quality Level 2 in
[REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`RVO2 Library Java` uses [Semantic Versioning 2.0.0](https://semver.org/), and
is at version `2.1.0`. The version is declared in `CMakeLists.txt` and
`MODULE.bazel`.

### Version Stability [1.ii]

`RVO2 Library Java` is at a stable version (`2.1.0`), which is greater than
`1.0.0`.

### Public API Declaration [1.iii]

The public API of `RVO2 Library Java` is the set of types declared in the
[`edu.unc.cs.gamma.rvo`](rvo/src/main/java/edu/unc/cs/gamma/rvo/) package:
`Simulator`, `AgentParameters`, and `Line`. Internal implementation details in
`Agent`, `KdTree`, `Obstacle`, and `RVOMath` are not part of the public API.

### API Stability Within a Released Version [1.iv]

The public API is stable across patch and minor versions. Breaking API changes
are only introduced in new major versions.

### Binary Compatibility Within a Released Version [1.v]

Binary compatibility is maintained across patch versions within the same
`major.minor` release series. New major or minor versions may introduce
binary-incompatible changes.

## Change Control Process [2]

### Change Requests [2.i]

All changes to `RVO2 Library Java` are submitted through pull requests on
[GitHub](https://github.com/snape/RVO2-Java). Direct commits to the `main`
branch are not permitted outside of automated tooling.

### Contributor Origin [2.ii]

All contributors must sign off on their commits using the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/),
enforced via the DCO GitHub App on every pull request.

### Peer Review Policy [2.iii]

As a single-maintainer project, all changes are reviewed by the primary
maintainer [@snape](https://github.com/snape) before merging.

### Continuous Integration [2.iv]

CI is performed via GitHub Actions on every push and pull request to `main`,
and on a daily schedule:

- [`.github/workflows/ci.yml`](.github/workflows/ci.yml): builds and tests
  with Bazel, CMake, and Gradle on AlmaLinux, Arch Linux, Fedora, openSUSE,
  and Ubuntu (amd64 and arm64), and macOS (arm64); CMake and Gradle
  additionally on Alpine Linux (amd64) and Windows (amd64). CMake builds
  enable `BUILD_DOCUMENTATION`, `BUILD_TESTING`, and `WARNINGS_AS_ERRORS`.
- [`.github/workflows/codeql.yml`](.github/workflows/codeql.yml): runs GitHub
  CodeQL semantic code analysis for Java on a weekly schedule.

### Documentation Policy [2.v]

Changes are documented through commit messages and pull request descriptions
on GitHub.

## Documentation [3]

### Feature Documentation [3.i]

All features of `RVO2 Library Java` are documented in the Javadoc markup
within the [`edu.unc.cs.gamma.rvo`](rvo/src/main/java/edu/unc/cs/gamma/rvo/)
package, including a usage guide, parameter overview, and example code. Three
annotated example programs
([`blocks/src/main/java/edu/unc/cs/gamma/rvo/examples/Blocks.java`](blocks/src/main/java/edu/unc/cs/gamma/rvo/examples/Blocks.java),
[`circle/src/main/java/edu/unc/cs/gamma/rvo/examples/Circle.java`](circle/src/main/java/edu/unc/cs/gamma/rvo/examples/Circle.java),
[`roadmap/src/main/java/edu/unc/cs/gamma/rvo/examples/Roadmap.java`](roadmap/src/main/java/edu/unc/cs/gamma/rvo/examples/Roadmap.java))
demonstrate typical usage patterns covering agents, obstacles, and step-wise
simulation.

### Public API Documentation [3.ii]

All public API elements are documented with Javadoc markup. HTML documentation
is generated from the source using `cmake -DBUILD_DOCUMENTATION=ON`,
`bazel build`, or `./gradlew javadoc`, and installed with the library.

### License [3.iii]

Source code is licensed under the
[Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0).
Documentation is licensed under the
[Creative Commons Attribution-ShareAlike 4.0 International (CC-BY-SA-4.0)](https://creativecommons.org/licenses/by-sa/4.0/)
Public License.

The project uses the [REUSE Specification](https://reuse.software/) for license
compliance. Every source file contains machine-readable SPDX license and
copyright headers, and license texts are provided in the [`LICENSES/`](LICENSES/)
directory. License compliance is verified by the `reuse` pre-commit hook.

### Copyright Statement [3.iv]

Copyright is held by the University of North Carolina at Chapel Hill. All
source files include the SPDX copyright notice:

```
SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
```

### Quality Declaration [3.v]

This document is the quality declaration for `RVO2 Library Java` and is linked
from the project README.

## Testing [4]

### Feature Testing [4.i]

Three system-level simulation scenarios form the test suite, run via `ctest`
(CMake), `bazel test` (Bazel), and `./gradlew test` (Gradle):

- **Blocks**: 100 agents split into four groups navigating through a narrow
  passage formed by four obstacles.
- **Circle**: 250 agents initially distributed on a circle moving to their
  antipodal positions.
- **Roadmap**: The Blocks scenario augmented with roadmap-based global path
  planning.

These scenarios exercise all documented features of the library, including
agent management, obstacle processing, the k-D tree spatial index, and
step-wise simulation.

### Public API Testing [4.ii]

The three simulation scenarios collectively exercise all major public API
functions of `Simulator`, `AgentParameters`, and `Line`. Dedicated unit tests
for individual API functions are not currently present.

### Coverage [4.iii]

Code coverage is not currently tracked. The three simulation scenarios provide
broad functional coverage of the library, but no formal coverage measurement
or policy is in place. This is a known gap relative to Quality Level 2
requirements.

### Performance [4.iv]

No formal performance regression tests are in place. The simulation scenarios
implicitly exercise performance characteristics of the library (e.g., the
Circle scenario with 250 agents, and the Blocks scenario with obstacle
processing), but no automated performance benchmarks are run in CI.

### Linters and Static Analysis [4.v]

The following linters and static analysis tools are enforced, with all warnings
treated as errors in CI:

- **checkstyle**: code style enforcement via
  [`config/checkstyle/checkstyle.xml`](config/checkstyle/checkstyle.xml)
- **ErrorProne**: static analysis via the `net.ltgt.errorprone` Gradle plugin
- **NullAway**: null safety enforcement via ErrorProne with
  `NullAway:AnnotatedPackages = edu.unc.cs.gamma.rvo`
- **buildifier**: Bazel file formatting via [`.buildifier.json`](.buildifier.json)
- **CodeQL**: GitHub's semantic code analysis for Java via
  [`.github/workflows/codeql.yml`](.github/workflows/codeql.yml)
- **pre-commit hooks**: `codespell` (spell checking), `yamllint`, `actionlint`
  (workflow validation), REUSE compliance, case-conflict detection, and
  trailing-whitespace removal via [`.pre-commit-config.yaml`](.pre-commit-config.yaml)

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

None.

### Optional Direct Runtime ROS Dependencies [5.ii]

None.

### Direct Runtime non-ROS Dependencies [5.iii]

- **Java Standard Library**: provided by the Java runtime (OpenJDK 21 or
  later). This is a de facto standard with no quality level concerns.
- **Apache Commons Math** (`org.apache.commons:commons-math3:3.6.1`): provides
  the `Vector2D` type used throughout the public API. This is a widely-used,
  well-maintained Apache library.
- **JSpecify** (`org.jspecify:jspecify:1.0.0`): provides standard null
  annotations used for static null safety analysis. This is a compile-time
  dependency only.

## Platform Support [6]

`RVO2 Library Java` is built and tested continuously via GitHub Actions on the
following platforms:

| Platform       | Architecture | Build System         |
|----------------|--------------|----------------------|
| AlmaLinux 10   | amd64        | Bazel, CMake, Gradle |
| Alpine Linux   | amd64        | CMake, Gradle        |
| Arch Linux     | amd64        | Bazel, CMake, Gradle |
| Fedora         | amd64        | Bazel, CMake, Gradle |
| openSUSE Leap  | amd64        | Bazel, CMake, Gradle |
| Ubuntu         | amd64, arm64 | Bazel, CMake, Gradle |
| macOS          | arm64        | Bazel, CMake, Gradle |
| Windows        | amd64        | CMake, Gradle        |

## Security [7]

### Vulnerability Disclosure Policy [7.i]

The security policy is documented in
[`.github/SECURITY.md`](.github/SECURITY.md). Vulnerability reports should be
sent to [geom@cs.unc.edu](mailto:geom@cs.unc.edu). The current release is
supported with security updates when practical.
