# QR Code reader

## Description
**Note: Assumes that the Raspberry Pi camera v2 is used**
Reads images from `/camera/image_raw` topic and publishes the QR code positions, normal vectors and orientations to `/qr_code` topic. Also fetches QR code data from `/get_qr_codes` service every five seconds to update the cache.

## Requirements
- numpy
- opencv
- opencv-contrib-python
- Requires that transform from odom to base_footprint is available
- Requires a publisher to `/camera/image_raw` and a service `/get_qr_codes`

## Parameters
| Parameter    | Type   | Default | Description                                                                                                                                                                                       |
| ------------ | ------ | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| tf_threshold | double | 0.01    | Time threshold for dropping data because latest transform(position) was too old. If a negative value is used, all transformations are used and no data is dropped at the cost of reduced accuracy |
| qr_code_size | double | 0.2     | QR code side length in meters                                                                                                                                                                     |

## Usage
```
ros2 run qr_code_reader launch
```