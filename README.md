# SnakeRaven-Project
The fully integrated SnakeRaven code including an updated SnakeRaven controller with control capability for dual arm teleoperation, vision system supporting hand-eye calibration and automated motion using waypoint navigation for dual arms.

## Prerequisite installation :
The snake_raven_controller and vision_system_snakeraven uses Eigen to compute the kinematics and VPC control algorithms. To install this do:
1. Go to http://eigen.tuxfamily.org/index.php?title=Main_Page#Download to get the most recent Eigen. 
2. Download the zip, extract and find the subfolder "Eigen" and subfolder "unsupported"
3. In snake_raven_controller/ and vision_system_snakeraven/ create the folder 'include' and paste 'Eigen' and 'unsupported' into the include folder.

## Project builds on Raven II software:
1. **uw-biorobotics/raven2** : [This code](https://github.com/uw-biorobotics/raven2) is the main RAVEN software to connect to (release 18_05). The modified files can be found in this repository's [raven_2](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/tree/main/raven_2) folder
2. My original works: [**snake_raven_controller**](https://github.com/Andrew-Raz-ACRV/snake_raven_controller) and [**vision_system_snakeraven**](https://github.com/Andrew-Raz-ACRV/vision_servo_control_snakeraven)

## How to use SnakeRaven
QUT has an assembled SnakeRaven tool, if using the RAVEN II computer at QUT skip to step 3. Instructions, CAD files and other resources to create your own SnakeRaven can be found in the appendix of my [thesis](https://eprints.qut.edu.au/235042/)

1. **Download** : On the Raven II computer ensure you have the packages snake_raven_controller/ and vision_system_snakeraven/ from this repository and place the folder in the Raven II folder e.g. home/raven_18_05

2. **Update** : Go to raven_18_05/raven_2 and update its contents with the three folders: /src /msg /include in the contents of folder [raven_2](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/tree/main/raven_2)

3. **Make** : Open a new terminal and cd to the Raven II directory. Run catkin_make to compile the new content and modifications
Note: step "source devel/setup.bash" can be unnecessary as it is being executed in .bashrc at start up
```
cd raven_18_05
source devel/setup.bash
catkin_make
```

4. **Run the Raven II** : With snakeraven instruments or without any on the Raven II, roslaunch the robot. Press the e-stop, twist release and press the silver reset to go through homing. 
```
roslaunch raven_2 raven_2.launch
```

5. **Velocity Joint Control mode** : Press 'm' to change mode and press '2' to start the velocity joint control mode, press the e-stop, twist release and press the silver reset for the mode change to occur.

6. **Run talkerSnakeRaven** : Open a new terminal and run command:
```
rosrun snake_raven_controller talkersnakeraven
```

7. **Run endoscopic camera node** : Open a new terminal and run command:
```
rosrun cv_camera cv_camera_node
```
Note: depending on your system you may need to change the camera parameter to device number #:
```
rosparam set cv_camera/device_id #
```
Optionally, You can check the view of the image feed with command
```
rosrun image_view image_view image:=/cv_camera/image_raw
```
Optionally for a different device # you can also rename the camera node but beaware this changes the topic name to subscribe to:
```
rosrun cv_camera cv_camera_node __name:=external _device_id=#
```

8. **Run endoscopic camera computer vision node** : Open a new terminal and run command:
```
rosrun vision_system_snakeraven imageprocessor
```

9. **Selection Menu** : All interaction is conducted in the snake_raven_controller node

_0. **Calibration** - this moves the arms to the required configuration given by the next user choice:

0. right arm, 1. left arm or 2. dual arm. 

In each of these cases the selected dominate arm or both arms are moved to be perpendicular to the table and the rest are moved aside. It will pop up a message saying that you can insert the SnakeRaven tool onto the tool holder. You can use the keyboard to adjust the joints of the robot inividually particularly to insert the SnakeRaven tool to mesh with the robot. Good Calibration is when SnakeRaven is neutral and perpendicular to the table as seen in the SnakeRaven [image](https://github.com/Andrew-Raz-ACRV/snake_raven_controller/blob/master/FrontCoverSnake2.png).

_1. **Joint Control** - this allows you to control the robot joints individually with the keyboard without calibration

**Joint Control Keyboard Mapping:**
See file [Keyboard_interactions.cpp](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/snake_raven_controller/src/Keyboard_interactions.cpp) 

Left Arm
- Shoulder +/-:      1/q
- Elbow +/-:         2/w
- Z Insertion +/-:   3/e
- Tool Rotation +/-: 4/r
- Wrist +/-:         5/t
- Grasp 1 +/-:       6/y
- Grasp 2 +/-:       7/u

Right Arm
- Shoulder +/-:      a/z
- Elbow +/-:         s/x
- Z Insertion +/-:   d/c
- Tool Rotation +/-: f/v
- Wrist +/-:         g/b
- Grasp 1 +/-:       h/n
- Grasp 2 +/-:       j/m

_2. **Teleoperation** - this allows you to control the SnakeRaven endeffector via keyboard but only after calibration. It has some additional code in the Raven_Controller class to:
- record teleoperation control onto a .csv file in the home folder

**Endeffector Keyboard Mapping:**
See file [Keyboard_interactions.cpp](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/snake_raven_controller/src/Keyboard_interactions.cpp) 
Dominant arm mapping
- Z+     q
- Z-     e
- Y+     w
- Y-     s
- X+     d
- X-     a
- freeze z
- pan +  f
- pan -  h
- tilt + t
- tilt - g
- z_rot+ y
- z_rot- r

Additional for right arm in dual arm setting:
- Z+     4
- Z-     6
- Y+     5
- Y-     2
- X+     3
- X-     4
- freeze z
- pan +  7
- pan -  9
- tilt + /
- tilt - 8
- z_rot+ -
- z_rot- *

_3. **Reset** - This moves both arms back to their perpendicular starting point as selected after calibration

_4. **Hand-eye calibration** - Warning: The outcome of this process is not very accurate and by default the hand-eye calibration is already set. This mode only supports the right arm configuration assuming that is where the camera is attached.
1. Put an [aruco marker](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/vision_system_snakeraven/ArucoMarker1.png) in the camera field of view and keep it stationary
2. The program starts recording data points immediantely so the user must teleoperate the robot until the data collection is complete. After completion, the new camera to tool transform is displayed.

_5. **IBVS assisted Teleoperation** - Best Teleoperation demo but only supports the right arm. This allows you to control the SnakeRaven endeffector via keyboard relative to the camera view with the IBVS assist support (see how that works in my [thesis](https://eprints.qut.edu.au/235042/)) but only after calibration. It has some additional code in the Raven_Controller class to:
- record teleoperation control onto a .csv file in the home folder

**Endeffector Keyboard Mapping relative to camera view:**
- 1      Toggles turning on or off the IBVS assist function
- Z+     w  forward
- Z-     s  retreat
- Y+     e  down
- Y-     q  up
- X+     d  right
- X-     a  left
- freeze z  toggle stop motion

Additional rotation under manual orientation control (without IBVS assist)
- X +    t  bend up
- X -    g  bend down
- Y +    f  bend left
- Y -    h  bend right

_6. **Waypoint Navigation** - The only fully autonomous mode. Under any arm configuration (including dual arm), this will move the tool down by a distance and begin tracing squares in space. It allows snakeraven to move either arm autonomously to a desired set of waypoints defined in functions in [code](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/snake_raven_controller/src/Waypoint_Task_process.cpp)

## Contact
This code is written by Andrew Razjigaev. If there are queries you can contact him via email: andrew_razjigaev@outlook.com
