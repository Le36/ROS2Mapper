# IO Node

## Description
Receives input from the user via a CLI and controls the functions of the TurtleBot3 by publishing messages to appropriate topics.

Has three different views for different functions:

### Main menu
| Command | Publishes to topic        | Published value | Description                         |
| ------- | ------------------------- | --------------- | ----------------------------------- |
| 1       | `/autonomous_exploration` | "1"             | Starts the autonomous exploration   |
| 2       | `/autonomous_exploration` | "2"             | Stops the autonomous exploration    |
| 3       | N/A                       | N/A             | Changes view to QR code menu        |
| 4       | N/A                       | N/A             | Changes view to manual control menu |
| CTRL+C  | N/A                       | N/A             | Quits the program                   |

### QR code menu
| Command | Publishes to topic        | Published value | Description                        |
| ------- | ------------------------- | --------------- | ---------------------------------- |
| 1-9     | `/qr_navigator`           | QRCode(...)     | Navigates to QR code with given ID |
| 0       | `/autonomous_exploration` | "2"             | Stops the autonomous exploration   |
| m       | N/A                       | N/A             | Return to main menu                |
| CTRL+C  | N/A                       | N/A             | Quits the program                  |

### Manual control menu
| Command      | Publishes to topic | Published value | Description                              |
| ------------ | ------------------ | --------------- | ---------------------------------------- |
| w            | `/cmd_vel`         | Twist(...)      | Increase linear velocity (move forward)  |
| x            | `/cmd_vel`         | Twist(...)      | Decrease linear velocity (move backward) |
| a            | `/cmd_vel`         | Twist(...)      | Increase angular velocity (turn left)    |
| w            | `/cmd_vel`         | Twist(...)      | Increase angular velocity (turn right)   |
| space key, s | `/cmd_vel`         | Twist(...)      | Stop all movement                        |
| m            | N/A                | N/A             | Return to main menu                      |
| CTRL+C       | N/A                | N/A             | Quits the program                        |

## Dependencies
- rclpy
- geometry_msgs
- interfaces

## Topics
| Publish/Subscribe | Topic                     | Type                      | Description                                                              |
| ----------------- | ------------------------- | ------------------------- | ------------------------------------------------------------------------ |
| Subscribe         | `/qr_code_list`           | interfaces/msg/QRCodeList | Updates internal QR code list when a new list is published to this topic |
| Subscribe         | `/log`                    | std_msgs/msg/String       | Prints the data that is published here                                   |
| Publish           | `/autonomous_exploration` | std_msgs/msg/String       | Publishes the commands for the exploration node                          |
| Publish           | `/cmd_vel`                | geometry_msgs/msg/Twist   | Publishes the movement when using manual control                         |

## Usage
```
ros2 run io_node control
```