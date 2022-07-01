# Camera node

## Description
Reads images from the camera and publishes them to `/camera/image_raw` five times in a second

## Requirements
- opencv

## Topics
| Publish/Subscribe | Topic               | Type                  | Description                                            |
| ----------------- | ------------------- | --------------------- | ------------------------------------------------------ |
| Publish           | `/camera/image_raw` | sensor_msgs/msg/Image | Publishes images from the camera five times per second |

## Usage
```
ros2 run camera_node launch
```