[![Test diagnostics](https://img.shields.io/github/actions/workflow/status/ros/diagnostics/test.yaml?label=test&style=flat-square)](https://github.com/ros/diagnostics/actions/workflows/test.yaml) [![Lint diagnostics](https://img.shields.io/github/actions/workflow/status/ros/diagnostics/lint.yaml?label=lint&style=flat-square)](https://github.com/ros/diagnostics/actions/workflows/lint.yaml) [![ROS2 Humble](https://img.shields.io/ros/v/humble/diagnostics.svg?style=flat-square)](https://index.ros.org/r/diagnostics/#humble) [![ROS2 Jazzy](https://img.shields.io/ros/v/jazzy/diagnostics.svg?style=flat-square)](https://index.ros.org/r/diagnostics/#jazzy) [![ROS2 Kilted](https://img.shields.io/ros/v/kilted/diagnostics.svg?style=flat-square)](https://index.ros.org/r/diagnostics/#kilted) [![ROS2 Rolling](https://img.shields.io/ros/v/rolling/diagnostics.svg?style=flat-square)](https://index.ros.org/r/diagnostics/#rolling) 

# Overview

The diagnostics system collects information about hardware drivers and robot hardware to make them available to users and operators.
The diagnostics system contains tools to collect and analyze this data.

The diagnostics system is build around the `/diagnostics` topic. The topic is used for `diagnostic_msgs/DiagnosticArray` messages.
It contains information about the device names, status, and values.

It contains the following packages:

- [`diagnostic_aggregator`](/diagnostic_aggregator/): Aggregates diagnostic messages from different sources into a single message.
- [`diagnostic_analysis`](/diagnostics/): *Not ported to ROS2 yet* **#contributions-welcome**
- [`diagnostic_common_diagnostics`](/diagnostic_common_diagnostics/): Predefined nodes for monitoring the Linux and ROS system.
- [`diagnostic_updater`](/diagnostic_updater/): Base classes to publishing custom diagnostic messages for Python and C++.
- [`self_test`](/self_test/): Tools to perform self tests on nodes.

## Collecting diagnostic data

At the points of interest, i.e. the hardware drivers, the diagnostic data is collected.
The data must be published on the `/diagnostics` topic.
In the `diagnostic_updater` package, there are base classes to simplify the creation of diagnostic messages.

## Aggregation

The `diagnostic_aggregator` package provides tools to aggregate diagnostic messages from different sources into a single message. It has a plugin system to define the aggregation rules.

## Visualization

Outside of this repository, there is [`rqt_robot_monitor`](https://index.ros.org/p/rqt_robot_monitor/) to visualize diagnostic messages that have been aggregated by the `diagnostic_aggregator`.

Diagnostics messages that are not aggregated can be visualized by [`rqt_runtime_monitor`](https://index.ros.org/p/rqt_runtime_monitor/).

# Target Distribution

- **Rolling Ridley** by the [`ros2` branch](https://github.com/ros/diagnostics/tree/ros2)
- **Humble Hawksbill** by the [`ros2-humble` branch](https://github.com/ros/diagnostics/tree/ros2-humble)
- **Jazzy Jalisco** by the [`ros2-jazzy` branch](https://github.com/ros/diagnostics/tree/ros2-jazzy)
- **Kilted Kaiju** by the [`ros2-kilted` branch](https://github.com/ros/diagnostics/tree/ros2-kilted)

## Workflow

New features are to be developed in custom branches and then merged into the `ros2` branch.

From there, the changes are backported to the other branches.

## Backport Tooling

This tool has proven to be useful: [backport](https://www.npmjs.com/package/backport)

Use this command to port a given PR of `PR_NUMBER` to the other branches:

```bash
backport --pr PR_NUMBER -b ros2-humble ros2-jazzy ros2-kilted
```

## Versioning and Releases

- (**X**.0.0) We use the major version number to indicate a breaking change.
- (0.**Y**.0) The minor version number is used to differentiate between different ROS distributions:
  - x.**0**.z: Humble Hawksbill
  - x.**2**.z: Jazzy Jalisco
  - x.**3**.z: Kilted Kaiju
  - x.**4**.z: Rolling Ridley
  - (Future releases will receive x.**4**.z and rolling will then be x.**5**.z)
- (0.0.**Z**) The patch version number is used for changes in the current ROS distribution that do not affect the API.

<!-- ## Buildfarm Statuses

### diagnostic_aggregator

|         | H | I | J | R |
| ------- | - | - | - | - |
| src, ubuntu | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hsrc_uJ__diagnostic_aggregator__ubuntu_jammy__source&style=ball-32x32)](https://build.ros2.org/job/Hsrc_uJ__diagnostic_aggregator__ubuntu_jammy__source) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Isrc_uJ__diagnostic_aggregator__ubuntu_jammy__source&style=ball-32x32)](https://build.ros2.org/job/Isrc_uJ__diagnostic_aggregator__ubuntu_jammy__source) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jsrc_uN__diagnostic_aggregator__ubuntu_noble__source&style=ball-32x32)](https://build.ros2.org/job/Jsrc_uN__diagnostic_aggregator__ubuntu_noble__source) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rsrc_uN__diagnostic_aggregator__ubuntu_noble__source&style=ball-32x32)](https://build.ros2.org/job/Rsrc_uN__diagnostic_aggregator__ubuntu_noble__source) |
| src, rhel | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hsrc_el8__diagnostic_aggregator__rhel_8__source&style=ball-32x32)](https://build.ros2.org/job/Hsrc_el8__diagnostic_aggregator__rhel_8__source) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Isrc_el9__diagnostic_aggregator__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Isrc_el9__diagnostic_aggregator__rhel_9__source) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jsrc_el9__diagnostic_aggregator__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Jsrc_el9__diagnostic_aggregator__rhel_9__source) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rsrc_el9__diagnostic_aggregator__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Rsrc_el9__diagnostic_aggregator__rhel_9__source) |
| bin, ubuntu, amd64 | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__diagnostic_aggregator__ubuntu_jammy_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_uJ64__diagnostic_aggregator__ubuntu_jammy_amd64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__diagnostic_aggregator__ubuntu_jammy_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_uJ64__diagnostic_aggregator__ubuntu_jammy_amd64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__diagnostic_aggregator__ubuntu_noble_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_uN64__diagnostic_aggregator__ubuntu_noble_amd64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__diagnostic_aggregator__ubuntu_noble_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_uN64__diagnostic_aggregator__ubuntu_noble_amd64__binary) |
| bin, ubuntu, arm64 | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__diagnostic_aggregator__ubuntu_jammy_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_ujv8_uJv8__diagnostic_aggregator__ubuntu_jammy_arm64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_ujv8_uJv8__diagnostic_aggregator__ubuntu_jammy_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_ujv8_uJv8__diagnostic_aggregator__ubuntu_jammy_arm64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__diagnostic_aggregator__ubuntu_noble_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_unv8_uNv8__diagnostic_aggregator__ubuntu_noble_arm64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_unv8_uNv8__diagnostic_aggregator__ubuntu_noble_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_unv8_uNv8__diagnostic_aggregator__ubuntu_noble_arm64__binary) |
| bin, rhel | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_rhel_el864__diagnostic_aggregator__rhel_8_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_rhel_el864__diagnostic_aggregator__rhel_8_x86_64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_rhel_el964__diagnostic_aggregator__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_rhel_el964__diagnostic_aggregator__rhel_9_x86_64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_rhel_el964__diagnostic_aggregator__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_rhel_el964__diagnostic_aggregator__rhel_9_x86_64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_rhel_el964__diagnostic_aggregator__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_rhel_el964__diagnostic_aggregator__rhel_9_x86_64__binary) |

### diagnostic_common_diagnostics

|         | H | I | J | R |
| ------- | - | - | - | - |
| src, ubuntu | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hsrc_uJ__diagnostic_common_diagnostics__ubuntu_jammy__source&style=ball-32x32)](https://build.ros2.org/job/Hsrc_uJ__diagnostic_common_diagnostics__ubuntu_jammy__source) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Isrc_uJ__diagnostic_common_diagnostics__ubuntu_jammy__source&style=ball-32x32)](https://build.ros2.org/job/Isrc_uJ__diagnostic_common_diagnostics__ubuntu_jammy__source) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jsrc_uN__diagnostic_common_diagnostics__ubuntu_noble__source&style=ball-32x32)](https://build.ros2.org/job/Jsrc_uN__diagnostic_common_diagnostics__ubuntu_noble__source) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rsrc_uN__diagnostic_common_diagnostics__ubuntu_noble__source&style=ball-32x32)](https://build.ros2.org/job/Rsrc_uN__diagnostic_common_diagnostics__ubuntu_noble__source) |
| src, rhel | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hsrc_el8__diagnostic_common_diagnostics__rhel_8__source&style=ball-32x32)](https://build.ros2.org/job/Hsrc_el8__diagnostic_common_diagnostics__rhel_8__source) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Isrc_el9__diagnostic_common_diagnostics__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Isrc_el9__diagnostic_common_diagnostics__rhel_9__source) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jsrc_el9__diagnostic_common_diagnostics__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Jsrc_el9__diagnostic_common_diagnostics__rhel_9__source) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rsrc_el9__diagnostic_common_diagnostics__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Rsrc_el9__diagnostic_common_diagnostics__rhel_9__source) |
| bin, ubuntu, amd64 | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__diagnostic_common_diagnostics__ubuntu_jammy_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_uJ64__diagnostic_common_diagnostics__ubuntu_jammy_amd64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__diagnostic_common_diagnostics__ubuntu_jammy_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_uJ64__diagnostic_common_diagnostics__ubuntu_jammy_amd64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__diagnostic_common_diagnostics__ubuntu_noble_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_uN64__diagnostic_common_diagnostics__ubuntu_noble_amd64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__diagnostic_common_diagnostics__ubuntu_noble_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_uN64__diagnostic_common_diagnostics__ubuntu_noble_amd64__binary) |
| bin, ubuntu, arm64 | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__diagnostic_common_diagnostics__ubuntu_jammy_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_ujv8_uJv8__diagnostic_common_diagnostics__ubuntu_jammy_arm64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_ujv8_uJv8__diagnostic_common_diagnostics__ubuntu_jammy_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_ujv8_uJv8__diagnostic_common_diagnostics__ubuntu_jammy_arm64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__diagnostic_common_diagnostics__ubuntu_noble_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_unv8_uNv8__diagnostic_common_diagnostics__ubuntu_noble_arm64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_unv8_uNv8__diagnostic_common_diagnostics__ubuntu_noble_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_unv8_uNv8__diagnostic_common_diagnostics__ubuntu_noble_arm64__binary) |
| bin, rhel | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_rhel_el864__diagnostic_common_diagnostics__rhel_8_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_rhel_el864__diagnostic_common_diagnostics__rhel_8_x86_64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_rhel_el964__diagnostic_common_diagnostics__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_rhel_el964__diagnostic_common_diagnostics__rhel_9_x86_64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_rhel_el964__diagnostic_common_diagnostics__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_rhel_el964__diagnostic_common_diagnostics__rhel_9_x86_64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_rhel_el964__diagnostic_common_diagnostics__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_rhel_el964__diagnostic_common_diagnostics__rhel_9_x86_64__binary) |

### diagnostic_updater

|         | H | I | J | R |
| ------- | - | - | - | - |
| src, ubuntu | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hsrc_uJ__diagnostic_updater__ubuntu_jammy__source&style=ball-32x32)](https://build.ros2.org/job/Hsrc_uJ__diagnostic_updater__ubuntu_jammy__source) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Isrc_uJ__diagnostic_updater__ubuntu_jammy__source&style=ball-32x32)](https://build.ros2.org/job/Isrc_uJ__diagnostic_updater__ubuntu_jammy__source) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jsrc_uN__diagnostic_updater__ubuntu_noble__source&style=ball-32x32)](https://build.ros2.org/job/Jsrc_uN__diagnostic_updater__ubuntu_noble__source) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rsrc_uN__diagnostic_updater__ubuntu_noble__source&style=ball-32x32)](https://build.ros2.org/job/Rsrc_uN__diagnostic_updater__ubuntu_noble__source) |
| src, rhel | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hsrc_el8__diagnostic_updater__rhel_8__source&style=ball-32x32)](https://build.ros2.org/job/Hsrc_el8__diagnostic_updater__rhel_8__source) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Isrc_el9__diagnostic_updater__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Isrc_el9__diagnostic_updater__rhel_9__source) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jsrc_el9__diagnostic_updater__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Jsrc_el9__diagnostic_updater__rhel_9__source) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rsrc_el9__diagnostic_updater__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Rsrc_el9__diagnostic_updater__rhel_9__source) |
| bin, ubuntu, amd64 | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__diagnostic_updater__ubuntu_jammy_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_uJ64__diagnostic_updater__ubuntu_jammy_amd64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__diagnostic_updater__ubuntu_jammy_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_uJ64__diagnostic_updater__ubuntu_jammy_amd64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__diagnostic_updater__ubuntu_noble_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_uN64__diagnostic_updater__ubuntu_noble_amd64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__diagnostic_updater__ubuntu_noble_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_uN64__diagnostic_updater__ubuntu_noble_amd64__binary) |
| bin, ubuntu, arm64 | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__diagnostic_updater__ubuntu_jammy_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_ujv8_uJv8__diagnostic_updater__ubuntu_jammy_arm64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_ujv8_uJv8__diagnostic_updater__ubuntu_jammy_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_ujv8_uJv8__diagnostic_updater__ubuntu_jammy_arm64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__diagnostic_updater__ubuntu_noble_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_unv8_uNv8__diagnostic_updater__ubuntu_noble_arm64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_unv8_uNv8__diagnostic_updater__ubuntu_noble_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_unv8_uNv8__diagnostic_updater__ubuntu_noble_arm64__binary) |
| bin, rhel | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_rhel_el864__diagnostic_updater__rhel_8_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_rhel_el864__diagnostic_updater__rhel_8_x86_64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_rhel_el964__diagnostic_updater__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_rhel_el964__diagnostic_updater__rhel_9_x86_64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_rhel_el964__diagnostic_updater__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_rhel_el964__diagnostic_updater__rhel_9_x86_64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_rhel_el964__diagnostic_updater__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_rhel_el964__diagnostic_updater__rhel_9_x86_64__binary) |

### self_test

|         | H | I | J | R |
| ------- | - | - | - | - |
| src, ubuntu | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hsrc_uJ__self_test__ubuntu_jammy__source&style=ball-32x32)](https://build.ros2.org/job/Hsrc_uJ__self_test__ubuntu_jammy__source) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Isrc_uJ__self_test__ubuntu_jammy__source&style=ball-32x32)](https://build.ros2.org/job/Isrc_uJ__self_test__ubuntu_jammy__source) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jsrc_uN__self_test__ubuntu_noble__source&style=ball-32x32)](https://build.ros2.org/job/Jsrc_uN__self_test__ubuntu_noble__source) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rsrc_uN__self_test__ubuntu_noble__source&style=ball-32x32)](https://build.ros2.org/job/Rsrc_uN__self_test__ubuntu_noble__source) |
| src, rhel | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hsrc_el8__self_test__rhel_8__source&style=ball-32x32)](https://build.ros2.org/job/Hsrc_el8__self_test__rhel_8__source) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Isrc_el9__self_test__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Isrc_el9__self_test__rhel_9__source) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jsrc_el9__self_test__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Jsrc_el9__self_test__rhel_9__source) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rsrc_el9__self_test__rhel_9__source&style=ball-32x32)](https://build.ros2.org/job/Rsrc_el9__self_test__rhel_9__source) |
| bin, ubuntu, amd64 | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__self_test__ubuntu_jammy_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_uJ64__self_test__ubuntu_jammy_amd64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__self_test__ubuntu_jammy_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_uJ64__self_test__ubuntu_jammy_amd64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__self_test__ubuntu_noble_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_uN64__self_test__ubuntu_noble_amd64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__self_test__ubuntu_noble_amd64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_uN64__self_test__ubuntu_noble_amd64__binary) |
| bin, ubuntu, arm64 | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_ujv8_uJv8__self_test__ubuntu_jammy_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_ujv8_uJv8__self_test__ubuntu_jammy_arm64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_ujv8_uJv8__self_test__ubuntu_jammy_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_ujv8_uJv8__self_test__ubuntu_jammy_arm64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_unv8_uNv8__self_test__ubuntu_noble_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_unv8_uNv8__self_test__ubuntu_noble_arm64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_unv8_uNv8__self_test__ubuntu_noble_arm64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_unv8_uNv8__self_test__ubuntu_noble_arm64__binary) |
| bin, rhel | [![Humble](https://build.ros2.org/buildStatus/icon?job=Hbin_rhel_el864__self_test__rhel_8_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Hbin_rhel_el864__self_test__rhel_8_x86_64__binary) | [![Iron](https://build.ros2.org/buildStatus/icon?job=Ibin_rhel_el964__self_test__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Ibin_rhel_el964__self_test__rhel_9_x86_64__binary) | [![Jazzy](https://build.ros2.org/buildStatus/icon?job=Jbin_rhel_el964__self_test__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Jbin_rhel_el964__self_test__rhel_9_x86_64__binary) | [![Rolling](https://build.ros2.org/buildStatus/icon?job=Rbin_rhel_el964__self_test__rhel_9_x86_64__binary&style=ball-32x32)](https://build.ros2.org/job/Rbin_rhel_el964__self_test__rhel_9_x86_64__binary) | -->

# License

The source code is released under a [BSD 3-Clause license](LICENSE).
