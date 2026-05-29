<!-- SPDX-FileCopyrightText: 2025-2026 FANUC America Corp.
     SPDX-FileCopyrightText: 2025-2026 FANUC CORPORATION

     SPDX-License-Identifier: Apache-2.0
-->
<!-- markdownlint-disable MD013 -->
# fanuc_driver

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-JAZZY_JALISCO-blue)](https://docs.ros.org/en/jazzy/index.html)
[![Ubuntu Noble](https://img.shields.io/badge/UBUNTU-24.04-orange)](https://documentation.ubuntu.com/release-notes/24.04/)
[![Main Branch](https://img.shields.io/badge/BRANCH-main-green)](https://github.com/FANUC-CORPORATION/fanuc_driver/tree/main)

![FANUC ROS 2 Control Driver](/images/FANUC_ros2_ControlDriver.jpg "FANUC ROS 2 Control Driver")

## About

This repository hosts the source code of the FANUC ROS 2 Driver project, a ros2_control high-bandwidth streaming driver.
This project will allow users to develop a ROS 2 application to control a FANUC virtual or real robot.

**Note**
The `main` branch targets **ROS 2 Jazzy Jalisco**.
Users of **ROS 2 Humble Hawksbill** should refer to the [humble](https://github.com/FANUC-CORPORATION/fanuc_driver/tree/humble) branch.

## Installation

See the [FANUC ROS 2 Driver Documentation](https://fanuc-corporation.github.io/fanuc_driver_doc/) for instructions.

## Licensing

The original FANUC ROS 2 Driver source code and associated documentation
including these web pages are Copyright (C) 2025-2026 FANUC America Corporation
and FANUC CORPORATION.

Any modifications or additions to source code or documentation
contributed to this project are Copyright (C) the contributor,
and should be noted as such in the comments section of the modified file(s).

FANUC ROS 2 Driver is licensed under
     [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0)

Exceptions:

- The sockpp library is licensed under the terms of the [BSD 3-Clause License](https://opensource.org/license/BSD-3-Clause).

- The readwriterqueue library is licensed under the terms of
  the [Simplified BSD License](https://opensource.org/license/BSD-2-Clause).

- The reflect-cpp and yaml-cpp libraries are licensed under the
  terms of the [MIT License](https://opensource.org/license/mit).

Please see the LICENSE folder in the root directory for the full texts of these licenses.
