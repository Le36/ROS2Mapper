# Interfaces

## Description
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
