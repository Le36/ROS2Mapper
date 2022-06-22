# Table of contents
- [Interfaces](#interfaces)
    - [interfaces/srv/QRCode](#interfacessrvqrcode)
    - [interfaces/srv/GetQRCodes](#interfacessrvgetqrcodes)
- [Topics](#topics)
    - [/qr_code](#qr_code)
    - [/log](#log)
    - [/camera/image_raw](#cameraimage_raw)
    - [/autonomous_exploration](#autonomous_exploration)
    - [/qr_navigator](#qr_navigator)
    - [/qr_list](#qr_list)
- [Services](#services)
    - [get_qr_codes](#get_qr_codes)

# API documentation
## Interfaces
### interfaces/srv/QRCode
- int32 id
    - Id of QR code
- float64[3] center
    - Center of the QR code in NAV2 coordinates
- float64[3] normal_vector
    - Vector pointing outwards of the QR code. Useful for getting the coordinates of the point in front of the QR code
- float64[4] rotation
    - Quaternion orientation facing the QR code

### interfaces/srv/GetQRCodes
- Request
    - std_msgs/msg/Empty
- Response
    - QRCode[] qr_codes
        - List of QR codes

## Topics
### /qr_code
- Type
    - interfaces/msg/QRCode
- Description
    - Every time a new QR code is found or an existing QR code has moved over 20cm or rotated over 20°, the QR code is published here
- Publishers
    - QRCodeReader
- Subscribers
    - MemoryNode

### /log
- Type
    - std_msgs/msg/String
- Description
    - Everything published here will be logged by the I/O node
- Publishers
    - QRCodeReader
- Subscribers
    - IONode

### /camera/image_raw
- Type
    - sensor_msgs/msg/Image
- Description
    - The camera node publishes images from the camera here with resolution 480x640
- Publishers
    - CameraNode
- Subscribers
    - QRCodeReader

### /autonomous_exploration
- Type
    - std_msgs/msg/String
- Description
    - The I/O node uses this topic to command the exploration node
- Publishers
    - IONode
- Subscribers
    - ExploreNode

### /qr_navigator
- Type
    - interfaces/msg/QRCode
- Description
    - The I/O node uses this topic to command the explore node to navigate to the given QR code
- Publishers
    - IONode
- Subscribers
    - ExploreNode

### /qr_list
- Type
    - interfaces/msg/QRCode
- Description
    - Every time a QR code is saved to the database it is published here
- Publishers
    - MemoryNode
- Subscribers
    - IONode

## Services
### get_qr_codes
- Type
    - interfaces/srv/GetQRCodes
- Description
    - This service can be called to get all of the QR codes from the database
- Service
    - MemoryNode
- Clients
    - N/A