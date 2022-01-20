## Pupil-ROS
The objective of this ROS package is to receive the various data streamed by Pupil Core and make them available as ROS topics.

## Prerequisites
Required Hardware:
- Pupil Core

Required software:
- Ubuntu 20.04
- ROS Noetic
- [Pupil Capture](https://docs.pupil-labs.com/core/#_1-put-on-pupil-core)

Required python packages:
- zmq
- msgpack
- opencv
- numpy
```
pip install pyzmq msgpack opencv-python
```

## Setup

### Pupil Capture
TODO

### Surface Tracking
1. Activate the `Surface Tracker` plugin under the plugins menu in Pupil Capture.
2. Follow the steps listed under: https://docs.pupil-labs.com/core/software/pupil-capture/#surface-tracking
TODO

## ROS Topics
A total of 4 topics are published:
- pupil_frame
- pupil_pupil
- pupil_gaze
- pupil_surface

### pupil_frame
This topic publishes the camera data from Pupil core. Three camera feeds are streamed by Pupil Core: the world camera (front camera) and the 2 eye cameras.
Structure:
- `topic`: identifies camera feed. 
    - "frame.world": front camera
    - "frame.eye.0"/"frame.eye.1": eye cameras
- `width`: width of camera data in pixel
- `height`: height of camera data in pixel
- `format`: color format. Default: bgr
- `image`: the image data. An example on formatting image data can be found under `pupil_connect/scripts/frame_sub.py`

### pupil_pupil
This topic directly reflects the structure of the [pupil datum format](https://docs.pupil-labs.com/developer/core/overview/#pupil-datum-format).

Care must be taken in which coordinate system data is mapped in. There are 3 coordinate systems, each with different conventions:
- 2D Image Space 
    - Origin: top left
    - Unit: pixels
    - Bounds: `x: [0, <image width>], y: [0, <image height>]`
- 2D Normalized Space
    - Normalized 2D image space
    - Origin: bottom left
    - Unit: Image width/height
    - Bounds: `x: [0, 1], y: [0, 1]`
- 3D Camera Space
    - Origin: Camera center
    - Bounds: n/a

These coordinate systems apply to pupil_gaze data as well. Which coordinate system each data field uses is listed in the above link. 

More information on the coordinate systems: https://docs.pupil-labs.com/core/terminology/#coordinate-system

### pupil_gaze
This topic directly reflects the structure of the [gaze datum format](https://docs.pupil-labs.com/developer/core/overview/#gaze-datum-format). Which coordinate systems are used for each data field is listed as well.  

`base_data` is not included yet. 

TODO: is data published monocular or binocular gaze data?

### pupil_surface
This topic directly reflects the structure of the [surface datum format](https://docs.pupil-labs.com/developer/core/overview/#surface-datum-format).

Surface coordinates are normalized with respect to the identified width and height of the surface with the orgin being bottom left. 

More information on the surface coordinate system: https://docs.pupil-labs.com/core/terminology/#surface-aoi-coordinate-system

## Project Structure
TODO
