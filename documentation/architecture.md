# Architecture #

Collection of different diagrams

### Architecture diagram ###

![Architecture diagram](images/ros2.drawio.png)

### Camera node ###

```
,------------------------------------------------------------.
|workspace.src.camera_node.camera_node.camera_node.CameraNode|
|------------------------------------------------------------|
|timer:                                                      |
|cap:                                                        |
|publisher:                                                  |
|bridge:                                                     |
|__init__(self):                                             |
|publish_image(self):                                        |
`------------------------------------------------------------'
                               |                              
                               |                              
                            ,----.                            
                            |Node|                            
                            |----|                            
                            `----'                            
```