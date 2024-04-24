# Nursing Robot MQP 23-24 ROS README
## Overview
This is a repository containing the ROS camera pipeline nodes for the 2023-2024 Nursing Robot MQP. This set of nodes allow two cameras to obtain positional data from AprilTags, fuses them for robust estimates, and publishes them to our Unity engine for use in the HoloLens. 
## Overall ROS Architecture
![ROS diagram drawio](https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/assets/91151120/e7608c1b-8a18-477a-a055-f23293d0e89d)

This diagram displays the relations between all of the ROS nodes involved in our project. Starting with the top left, the camera pipeline produces object data from objects marked with AprilTags along with camera position estimates using a static table AprilTag. The data is then broadcasted to our TF tree using a TF broadcaster to allow visualization in RViz. To publish to Unity there are two publishers we use: an intermediate publisher and a Unity publisher. The intermediate publisher is responsible for fusing the object estimates from each camera and publishing the combined estimate back to the TF tree. Then the Unity publishers takes the combined object estimates as well as the camera estimates and sends them to our Unity engine so the HoloLens can use them when drawing holographic overlays. The intermediate publisher also publishes different points on a given object such as publishing the center of a cup's handle, center of its bowl, etc. This is sent to the ROS topic /objects which is used for LocoBot object manipulation.
## Vision Pipeline
![Vision Pipeline drawio](https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/assets/91151120/c65de392-803a-4661-9b74-861a0ed24939)

This diagram shows the specifics of the vision pipeline. Due to delays and frame issues within Unity and the Hololens resulting from using a high resolution camera, we had to thread the image acquisition for both cameras. To start, the cameras need to localize themselves within the workspace which is done during a calibration sequence using the following AprilTag and QR code sheet.

![image](https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/assets/91151120/96086de8-ac3c-4a86-8a74-5877fd00218b)

This is done because the HoloLens utilizes a spatial anchor for localization which is necessary for it to place the digital twins of objects in the correct places. However there are noticeable inaccuracies with spatial anchors so we use this AprilTag to help localize everything on the ROS side in relation to the tag since the AprilTag will always be the same distance from the QR code which is used to place a Hololens spatial anchor. The relation of all objects to our AprilTag is called our adjust frame which allows us to use our `teleop.py` script to move around the position of all of the objects to better align the digital twins in the Hololens.
It is important to note that this is a one-shot calibration sequence so **once the cameras are calibrated, they cannot be moved.** By using OpenCV each camera is setup to acquire frames to detect AprilTags in each frame. Once an AprilTag is detected, we use PnP to calculate the centers of each tag. Each object has an AprilTag with its own tag ID to distinguish objects within our TF tree and publishers. Note: **We only use tag family 36h11. During inital testing we could not detect other tag families.** All of the object positions are then published through ROS for processing and publishing to all relevant nodes.

## Setup

### Software Requirements

Ensure that you are on a computer running Ubuntu 20.04 with ROS 1 Noetic installed

To run the camera nodes you will need install `AprilTag` from the following repository (**each person running the camera nodes needs to do this separately)**:

[AprilTags](https://github.com/AprilRobotics/apriltag.git)

You will also need OpenCV if not installed already. Use the command

```
sudo pip install opencv-python
```
if Python 3 is your default version, or
```
sudo pip3 install opencv-python
``` 
if you have both Python 2 and Python 3 installed on your system.

Here are the steps to ensure a connection with the HoloLens:
1. Install `ROS-TCP-Endpoint` from the following repository into your catkin workspace: [TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git)
2. Connect to the HiRo labs wifi, do not use WPI WiFi.
3. Use `ifconfig ` to obtain your ip address for the HoloLens user.

Before running you will need to ensure that you set up the cameras correctly in `pipeline.py`. An easy way to do this is by using ffmpeg. To install run the command 

```
sudo apt install ffmpeg
```

Then run the command 

```
ffmpeg -f v4l2 -video_size 640x480 -i /dev/video# -vf "format=yuv420p" -f sdl "Webcam Feed"
```

If you have a only laptop webcam and did not have the external cameras plugged in while booting up the "#" should be able to be replaced with a 2 or 4. If not, you will have to experiment with different numbers until you find the right one. The command will bring up a view of what the corresponding camera sees. This is useful to ensure that you know which camera is which and that each camera can see the calibration AprilTag.

To set the cameras, edit the following lines: [[https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/blob/0c1b8f5c86c135f94c74517d381832da40414e0c/src/pipeline.py#L340](https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/blob/93d46f2faa306de52ecfaef543fba2a1fa6e12f6/src/pipeline.py#L340-L341)](https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/blob/2d8502f1965630d6c34faaac103ce3b3ffa59dd8/src/pipeline.py#L340-L341)

### Running

Once you setup the workspace run the command

```
roslaunch ROS-Nursing-Robot-MQP-23-24 launch.launch
```

To move the adjust frames and line up the digital twins click into the terminal running the ROS nodes. The teleoperation script should be running by default. The commands to adjust are as follows:
- "w" and "s" to adjust in the x direction
- "a" and "d" to adjust in the y direction
- "q" and "e" to adjust in the z direction

The specific direction for each key depends on the orientation of the calibration tag so that will need to be figured out situationally.

Since the teleoperation script will be running by default you will need to press `space` to exit the teleoperation script.

### Miscellaneous Notes

If you want to add a new object to the workspace follow these steps:
1. Ensure that you use an AprilTag in the 36h11 family that is separate from the tag IDs already in use.
2. Refer to the following lines and follow the format to add a tag that the cameras can detect along with a corresponding object name. https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/blob/93d46f2faa306de52ecfaef543fba2a1fa6e12f6/src/pipeline.py#L73-L110
3. Add the object name to the following list. https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/blob/93d46f2faa306de52ecfaef543fba2a1fa6e12f6/src/pipeline.py#L222
4. Add the object name to the broadcaster in the intermediate publisher using the following format in `intermediate_publisher.py`. https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/blob/488107e449fc831e9eda7880ecc063fd470e78f8/src/intermediate_publisher.py#L118-L124
5. Use the object name in the following format to add objects to the unity publisher. https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/blob/488107e449fc831e9eda7880ecc063fd470e78f8/src/unity_publisher.py#L63-L68
