# Nursing Robot MQP 23-24 ROS README
## Overview
This is a repository containing the ROS camera pipeline nodes for the 2023-2024 Nursing Robot MQP. This set of nodes allow two cameras to obtain positional data from AprilTags, fuses them for robust estimates, and publishes them to our Unity engine for use in the HoloLens. 
## Overall ROS Architecture
![ROS diagram drawio](https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/assets/91151120/e7608c1b-8a18-477a-a055-f23293d0e89d)

This diagram displays the relations between all of the ROS nodes involved in our project. Starting with the top left, the camera pipeline produces object data from objects marked with AprilTags along with camera position estimates using a static table AprilTag. The data is then broadcasted to our TF tree using a TF broadcaster to allow visualization in RViz. To publish to Unity there are two publishers we use: an intermediate publisher and a Unity publisher. The intermediate publisher is responsible for fusing the object estimates from each camera and publishing the combined estimate back to the TF tree. Then the Unity publishers takes the combined object estimates as well as the camera estimates and sends them to our Unity engine so the HoloLens can use them when drawing holographic overlays. The intermediate publisher also publishes different points on a given object such as publishing the center of a cup's handle, center of its bowl, etc. This is sent to the ROS topic /objects which is used for LocoBot object manipulation.
## Vision Pipeline
![Vision Pipeline drawio](https://github.com/technoAl/ROS-Nursing-Robot-MQP-23-24/assets/91151120/c65de392-803a-4661-9b74-861a0ed24939)

This diagram shows the specifics of the vision pipeline. Due to delays and frame issues within Unity and the Hololens resulting from using a high resolution camera, we had to thread the image acquisition for both cameras. To start, the cameras need to localize themselves within the workspace which is done using the following AprilTag and QR code sheet. (insert image here) Placing the two near each other allows all of the objects
