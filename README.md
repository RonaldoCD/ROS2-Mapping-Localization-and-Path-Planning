# ROS2 NAVIGATION PROJECT: MAPPING, LOCALIZATION AND PATH PLANNING 

This repository uses the NAV2 package from ROS2 to implement a robot with navigation capabilities. Within their designated folders, you'll find launch and configuration files for mapping (using Cartographer), localization, path planning, and supplementary implemented nodes to enhance navigation of the robot through the maze.

## Mapping
* Launch file: [cartographer.launch.py](https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning/blob/main/project_mapping/launch/cartographer.launch.py)

![mapping](https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning/assets/73918490/2c6deb6e-feaa-4b17-8046-946461cae061)
![simulation_mapping](https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning/assets/73918490/7668a76f-5182-48bd-ac05-8b594b36f0f8)

## Localization

* Launch file: [localization.launch.py](https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning/blob/main/project_localization/launch/localization.launch.py)

[Screencast from 02-20-2024 05:12:42 PM.webm](https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning/assets/73918490/35c4cf87-640a-4c39-a557-45c341621054)

## Path Planning

* Launch file: [path_planning_server.launch.py](https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning/blob/main/project_path_planning/launch/path_planning_server.launch.py) - also launches localization nodes.

[Screencast from 02-20-2024 05:15:15 PM.webm](https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning/assets/73918490/ab06a889-b13f-4b52-8921-08d075a67d7c)

[Screencast from 02-20-2024 05:24:09 PM.webm](https://github.com/RonaldoCD/ROS2-Mapping-Localization-and-Path-Planning/assets/73918490/fbe7bec8-0225-4120-ac11-36e743ff194c)

