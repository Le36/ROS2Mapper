# Table of contents
- [Interfaces](#interfaces)
    - [Msg](#msg)
        - [interfaces/msg/QRCode](#interfacesmsgqrcode)
    - [Srv](#srv)
        - [interfaces/srv/GetQRCodes](#interfacessrvgetqrcodes)
- [Topics](#topics)
- [Services](#services)

# API documentation
## Interfaces
### Msg
#### interfaces/msg/QRCode
| Data          | Type       | Description                                                                                                      |
| ------------- | ---------- | ---------------------------------------------------------------------------------------------------------------- |
| id            | int32      | Id of QR code                                                                                                    |
| center        | float64[3] | Center of the QR code in NAV2 coordinates                                                                        |
| normal_vector | float64[3] | Vector pointing outwards of the QR code. Useful for getting the coordinates of the point in front of the QR code |
| rotation      | float64[4] | Quaternion orientation facing the QR code                                                                        |

### Srv
#### interfaces/srv/GetQRCodes
| Request/Response | Data     | Type               | Description      |
| ---------------- | -------- | ------------------ | ---------------- |
| Request          |          | std_msgs/msg/Empty |                  |
| Response         | qr_codes | QRCode[]           | List of QR codes |

## Topics
| Topic                   | Type                  | Publishers     | Subscribers    | Description                                                                                                                     |
| ----------------------- | --------------------- | -------------- | -------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| /qr_code                | interfaces/msg/QRCode | `QRCodeReader` | `MemoryNode`   | Every time a new QR code is found or an existing QR code has moved over 20cm or rotated over 20°, the QR code is published here |
| /log                    | std_msgs/msg/String   | `QRCodeReader` | `IONode`       | Everything published here will be logged by the I/O node                                                                        |
| /camera/image_raw       | sensor_msgs/msg/Image | `CameraNode`   | `QRCodeReader` | The camera node publishes images from the camera here with resolution 480x640                                                   |
| /autonomous_exploration | std_msgs/msg/String   | `IONode`       | `ExploreNode`  | The I/O node uses this topic to command the exploration node                                                                    |
| /qr_navigator           | interfaces/msg/QRCode | `IONode`       | `ExploreNode`  | The I/O node uses this topic to command the explore node to navigate to the given QR code                                       |
| /qr_list                | interfaces/msg/QRCode | `MemoryNode`   | `IONode`       | Every time a QR code is saved to the database it is published here                                                              |

## Services
| Service      | Type                      | Service      | Clients | Description                                                             |
| ------------ | ------------------------- | ------------ | ------- | ----------------------------------------------------------------------- |
| get_qr_codes | interfaces/srv/GetQRCodes | `MemoryNode` |         | This service can be called to get all of the QR codes from the database |
