# Memory node

## Description
Handles saving the QR codes into the database and publishes the list of QR codes every five seconds and every time the database is updated.  
Listens to `/qr_code` for new/updated QR codes and publishes the QR code list to `/qr_code_list`

## Dependencies
- numpy
- interfaces

## Topics
| Publish/Subscribe | Topic           | Type                      | Description                                                                                   |
| ----------------- | --------------- | ------------------------- | --------------------------------------------------------------------------------------------- |
| Subscribe         | `/qr_code`      | interfaces/msg/QRCode     | Updates/adds QR codes to the database when a QR code is published to this topic               |
| Publish           | `/qr_code_list` | interfaces/msg/QRCodeList | Publishes the list of QR codes every five seconds and every time the list of QR codes changes |


## Usage
```
ros2 run ros2mapper_memory_node listener
```