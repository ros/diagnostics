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

- **Rolling Ridley** and **Lyrical Luth** by the [`ros2` branch](https://github.com/ros/diagnostics/tree/ros2)
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

## Buildfarm Statuses

|  | Rolling | Lyric | Kilted | Jazzy | Humble |
| --- | - | - | - | - | - |
| `dev` | [![](https://build.ros2.org/job/Rdev__diagnostics__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Rdev__diagnostics__ubuntu_resolute_amd64/) | [![](https://build.ros2.org/job/Ldev__diagnostics__ubuntu_resolute_amd64/badge/icon)](https://build.ros2.org/job/Ldev__diagnostics__ubuntu_resolute_amd64/) | [![](https://build.ros2.org/job/Kdev__diagnostics__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__diagnostics__ubuntu_noble_amd64/) | [![](https://build.ros2.org/job/Jdev__diagnostics__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__diagnostics__ubuntu_noble_amd64/) | [![](https://build.ros2.org/job/Hdev__diagnostics__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__diagnostics__ubuntu_jammy_amd64/) |

# License

The source code is released under a [BSD 3-Clause license](LICENSE).
