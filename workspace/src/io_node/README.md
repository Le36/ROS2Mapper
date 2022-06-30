# IO Node

## Description
Receives input from the user via a CLI and controls the functions of the TurtleBot3 by publishing messages to appropriate topics.

Has three different views for different functions:

---

### Main menu
| Command | Publishes to topic      | Description                         |
| ------- | ----------------------- | ----------------------------------- |
| 1       | /autonomous_exploration | Starts the autonomous exploration   |
| 2       | /autonomous_exploration | Stops the autonomous exploration    |
| 3       | N/A                     | Changes view to QR code menu        |
| 4       | N/A                     | Changes view to manual control menu |
| CTRL+C  | N/A                     | Quits the program                   |

---

### QR code menu
| Command | Publishes to topic      | Description                        |
| ------- | ----------------------- | ---------------------------------- |
| 1-9     | /qr_navigator           | Navigates to QR code with given ID |
| 0       | /autonomous_exploration | Stops the autonomous exploration   |
| m       | N/A                     | Return to main menu                |
| CTRL+C  | N/A                     | Quits the program                  |

---

### Manual control menu
| Command      | Publishes to topic | Description                              |
| ------------ | ------------------ | ---------------------------------------- |
| w            | /cmd_vel           | Increase linear velocity (move forward)  |
| x            | /cmd_vel           | Decrease linear velocity (move backward) |
| a            | /cmd_vel           | Increase angular velocity (turn left)    |
| w            | /cmd_vel           | Increase angular velocity (turn right)   |
| space key, s | /cmd_vel           | Stop all movement                        |
| m            | N/A                | Return to main menu                      |
| CTRL+C       | N/A                | Quits the program                        |

Reads images from `/camera/image_raw` topic and publishes the QR code positions, normal vectors and orientations to `/qr_code` topic. Also fetches QR code data from `/get_qr_codes` service every five seconds to update the cache.

## Dependencies
- rclpy
- geometry_msgs                                                                                                                                                             |

## Usage
```
ros2 run io_node control
```