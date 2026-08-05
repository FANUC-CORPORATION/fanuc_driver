<!-- SPDX-FileCopyrightText: 2026 FANUC America Corp.
     SPDX-FileCopyrightText: 2026 FANUC CORPORATION

     SPDX-License-Identifier: Apache-2.0
-->
<!-- markdownlint-disable MD013 -->
# fanuc_rmi_controller_example

This example shows how to use fanuc_rmi_controller from python.

## Environment

* Robot model: CRX-10iA/L
* Controller: R-30iB Mini Plus or R-50iA.

**Note**
This example assumes that the all values in UTool[1] are set to 0 on the robot controller.

## How to use

1. Prepare the robot controller. (Set the robot controller mode to AUTO, reset alarms, etc.)
2. Run the fanuc_driver

   ```bash
   ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_ip:=*.*.*.* robot_model:=crx10ia_l gpio_config_path:=config/example_gpio_config_small.yaml
   ```

3. Run `rmi_example.py`

   ```bash
   ros2 run fanuc_rmi_controller_example rmi_example.py
   ```

4. The robot starts to move.

## Explanation

See main() function in the `scripts/rmi_example.py`.

For the detailed usage of RMI, please refer to the Remote Motion Interface manual.
