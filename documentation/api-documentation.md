# API documentation
## Interfaces

#### interfaces/msg/QRCode
| Data          | Type       | Description                                                                                                      |
| ------------- | ---------- | ---------------------------------------------------------------------------------------------------------------- |
| id            | int32      | Id of QR code                                                                                                    |
| center        | float64[3] | Center of the QR code in NAV2 coordinates                                                                        |
| normal_vector | float64[3] | Vector pointing outwards of the QR code. Useful for getting the coordinates of the point in front of the QR code |
| rotation      | float64[4] | Quaternion orientation facing the QR code                                                                        |

#### interfaces/msg/QRCodeList
| Data     | Type     | Description      |
| -------- | -------- | ---------------- |
| qr_codes | QRCode[] | List of QR codes |

## Topics
| Topic                   | Type                      | Publishers     | Subscribers             | Description                                                                                                                     |
| ----------------------- | ------------------------- | -------------- | ----------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| /qr_code                | interfaces/msg/QRCode     | `QRCodeReader` | `MemoryNode`            | Every time a new QR code is found or an existing QR code has moved over 20cm or rotated over 20°, the QR code is published here |
| /log                    | std_msgs/msg/String       | `QRCodeReader` | `IONode`                | Everything published here will be logged by the I/O node                                                                        |
| /camera/image_raw       | sensor_msgs/msg/Image     | `CameraNode`   | `QRCodeReader`          | The camera node publishes images from the camera here with resolution 480x640                                                   |
| /autonomous_exploration | std_msgs/msg/String       | `IONode`       | `ExploreNode`           | The I/O node uses this topic to command the exploration node                                                                    |
| /qr_navigator           | interfaces/msg/QRCode     | `IONode`       | `ExploreNode`           | The I/O node uses this topic to command the explore node to navigate to the given QR code                                       |
| /qr_code_list           | interfaces/msg/QRCodeList | `MemoryNode`   | `IONode` `QRCodeReader` | Called with the current list of QR codes five times per second and every time the database is updated                           |

## Quality of Services
| Topic                   | History   | Depth | Reliability | Durability | Deadline       | Lifespan       | Liveliness     | Lease Duration |
| ----------------------- | --------- | ----- | ----------- | ---------- | -------------- | -------------- | -------------- | -------------- |
| /qr_code                | Keep last | 10    | Reliable    | Volatile   | System default | System default | System default | System default |
| /log                    | Keep last | 10    | Reliable    | Volatile   | System default | System default | System default | System default |
| /camera/image_raw       | Keep last | 10    | Reliable    | Volatile   | System default | System default | System default | System default |
| /autonomous_exploration | Keep last | 5     | Reliable    | Volatile   | System default | System default | System default | System default |
| /qr_navigator           | Keep last | 5     | Reliable    | Volatile   | System default | System default | System default | System default |
| /qr_code_list           | Keep last | 10    | Reliable    | Volatile   | System default | System default | System default | System default |