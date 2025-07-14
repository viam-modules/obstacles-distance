# Obstacles Distance Module

This module provides a service for a segmenter that takes point clouds from a camera input and returns the average single closest point to the camera as a perceived obstacle. It is best for transient obstacle avoidance.

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "camera_name": "realsense-camera",
  "num_queries": 10
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `camera_name` | string | Optional  | The name of the default camera to use |
| `num_queries` | int    | Optional  | The number of queries to make by the camera before averaging, the default value is 10 |

#### Example Camera Configuration

```json
{
  "name": "camera-1",
  "api": "rdk:component:camera",
  "model": "viam:camera:realsense",
  "attributes": {
    "width_px": 640,
    "height_px": 480,
    "little_endian_depth": false,
    "serial_number": "",
    "sensors": [
      "depth",
      "color"
    ]
  }
}
```

#### Example Model Configuration

```json
{
  "name": "vision-1",
  "api": "rdk:service:vision",
  "model": "viam:vision:obstacles-distance",
  "attributes": {
    "camera_name": "camera-1",
    "num_queries": 10
  }
}
```
