
ROS Bag
========

Overview
#######

In this lab we are going to create a rosbag file filled with data streamed from Orbec Camera sensor.

While following the step-by-step tutorials, please take your time to think about what you are doing and what happens in each step, with the help of Google if necessary.

Preview: Next week we are going to learn how to write ROS scripts to send and receive messages; also, if time permits, we will create a simple service and implement a simple client-server routine.

Set up Drivers for Orbbec Astra Devices
#####

Ros wiki has a page dedicated to Orbbec Astra Devices: http://wiki.ros.org/astra_camera . Here you can find the instructions to install the required drivers.
The related  GitHub repository is  `here <https://github.com/orbbec/ros_astra_camera>`_.

Stream and record data
####

* Now let's plug the Orbec3D camera to your PC (https://orbbec3d.com/index/Product/info.html?cate=38&id=36) and run astra_camera nodes:
.. ::
  $ roslaunch astra_camera astra.launch
* Check the node and the topics generate by launching the previous file.
* Record streamed data. Let's create three different bags: all_topics_bag, rgb_only_bag, depth_only_bag. Use rosbash commands. 
.. ::
  $ rosbag record <continue>
* playback the recorded bags and visualize the output via 
.. ::
  $ rosrun rqt_image_view rqt_image_view
