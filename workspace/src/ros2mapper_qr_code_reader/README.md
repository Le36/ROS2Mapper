# QR Code reader

## Description
**Note: Assumes that the Raspberry Pi Camera v2 is used**  
This node reads images from `/camera/image_raw` topic and publishes the QR code positions, normal vectors, and orientations to the `/qr_code` topic. This node also updates its internal list of QR codes from `/qr_code_list`. QR codes farther than 3m away are ignored to prevent issues with the exploration and the low resolution of the images.

**Sources for the math**
- [Rotation matrices](https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations)
- [Quaternion -> Euler angles](shorturl.at/djB89)
- [Quaternion rotation of angle between two vectors](https://stackoverflow.com/a/1171995)
- [Pixel -> 3d point](https://math.stackexchange.com/a/4405154)

## Requirements
- numpy
- opencv
- opencv-contrib-python
- Requires that transform from odom to base_footprint is available
- interfaces

## Parameters
| Parameter    | Type   | Default | Description                                                                                                                                                                                           |
| ------------ | ------ | ------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| tf_threshold | double | 0.01    | Time threshold for dropping data because the latest transform(position) was too old. If a negative value is used, all transformations are used and no data is dropped at the cost of reduced accuracy |
| qr_code_size | double | 0.2     | QR code side length in meters                                                                                                                                                                         |

## Topics
| Publish/Subscribe | Topic               | Type                      | Description                                                                       |
| ----------------- | ------------------- | ------------------------- | --------------------------------------------------------------------------------- |
| Subscribe         | `/camera/image_raw` | sensor_msgs/msg/Image     | Reads images from this topic                                                      |
| Subscribe         | `/qr_code_list`     | interfaces/msg/QRCodeList | Updates internal QR code list when a new list is published to this topic          |
| Publish           | `/qr_code`          | interfaces/msg/QRCode     | Publishes new QR codes and QR codes that have moved over 20cm or rotated over 20° |
| Publish           | `/log`              | std_msgs/msg/String       | Publishes here when a QR code has moved/rotated or a new QR code was found        |


## Usage
```
ros2 run ros2mapper_qr_code_reader launch
```
