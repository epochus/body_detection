body_detection
==============
This is the main repository of our BWI Project for CS378: Autonomous Intelligent Robotics.
This project was last implemented in ROS Groovy Galapagos.

Maintained by: <br>
Jackson Wu <br>
Jordan Torres

Project Tutorial:
-----------------
NOTE: This package requires an installation of openni_tracker and sound_play. Both of these packages may be found in http://www.ros.org/wiki/. Video demo at http://www.youtube.com/watch?v=_cZ5n7Ngcg8&feature=youtu.be.

To Run:
Each in separate terminals,
```
$ rosrun sound_play soundplay_node.py
$ rosrun openni_tracker openni_tracker
$ rosrun body_scanning body_scanning.cpp
```
Given published transforms (/tf) for each body part, this node will interpret the data into the height, distance, and recent movement of the user for audio output.
