# ROS2 Mapper

[![CI](https://github.com/Le36/ros2-mapper/actions/workflows/main.yml/badge.svg)](https://github.com/Le36/ros2-mapper/actions/workflows/main.yml)
[![codecov](https://codecov.io/gh/Le36/ros2-mapper/branch/main/graph/badge.svg?token=8TI9BF62Q4)](https://codecov.io/gh/Le36/ros2-mapper)  
University of Helsinki, Ohjelmistotuotantoprojekti (Software Engineering Project), summer 2022

## Documents

- [Definition of done](documentation/definition-of-done.md)
- [Timesheet](documentation/timesheet.md)
- [Product and sprint backlogs](https://github.com/Le36/ros2-mapper/projects)
- [Sprint 3 burndown chart](documentation/images/sprint3.png)

## Project documentation
- [Install instructions](documentation/installation.md)
- [Architecture](documentation/architecture.md)
- [API documentation](documentation/api-documentation.md)

### Nodes
- [Camera node](workspace/src/ros2mapper_camera_node/)
- [Explore node](workspace/src/ros2mapper_explore_node/)
- [Interfaces](workspace/src/interfaces/)
- [IO node](workspace/src/ros2mapper_io_node/)
- [Memory node](workspace/src/ros2mapper_memory_node/)
- [Nav2 simple commander](workspace/src/nav2_simple_commander/)
- [QR Code reader](workspace/src/ros2mapper_qr_code_reader/)

### Running the project
In the Gazebo simulator
```
cd workspace
source install/setup.bash && source venv/bin/activate
./run.sh
```
On the physical robot
```
cd workspace
source install/setup.bash && source venv/bin/activate
IP=<TurtleBot3 IP address> ./run.sh
```

## Dev documents

- [Tutorials](documentation/tutorials.md)
- [TurtleBot3 image](https://drive.google.com/file/d/1JExsfCfhW8HvZbS-rrAKpXwOzQ3-d5AO/view?usp=sharing)

## Diaries

- [Juho](documentation/diaries/juho.md)
- [Kosti](documentation/diaries/kosti.md)
- [Leevi](documentation/diaries/leevi.md)
- [Oskari](documentation/diaries/oskari.md)
- [Sami](documentation/diaries/sami.md)
