# SnakeRaven-Project
The fully integrated SnakeRaven code including an updated SnakeRaven controller with control capability for dual arm teleoperation, vision system supporting hand-eye calibration and automated motion using waypoint navigation for dual arms. This project was completed in March 2023.

## Prerequisite installation :
The snake_raven_controller and vision_system_snakeraven uses Eigen to compute the kinematics and VPC control algorithms. To install this do:
1. Go to http://eigen.tuxfamily.org/index.php?title=Main_Page#Download to get the most recent Eigen. 
2. Download the zip, extract and find the subfolder "Eigen" and subfolder "unsupported"
3. In snake_raven_controller/ and vision_system_snakeraven/ create the folder 'include' and paste 'Eigen' and 'unsupported' into the include folder.

## Project builds on Raven II software:
1. **uw-biorobotics/raven2** : [This code](https://github.com/uw-biorobotics/raven2) is the main RAVEN software to connect to (release 18_05). Note that this project was made for the ROS kinetic release. The modified files can be found in this repository's [raven_2](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/tree/main/raven_2) folder
2. My original works: [**snake_raven_controller**](https://github.com/Andrew-Raz-ACRV/snake_raven_controller) and [**vision_system_snakeraven**](https://github.com/Andrew-Raz-ACRV/vision_servo_control_snakeraven)

## How to use SnakeRaven

0. **RAVEN II start up procedure at QUT**: On the bottom stack, turn on the 48V power and wait for 5 seconds. Then turn on the system power and toggle the power button in the 4th stack to turn on the computer. Login details are provided in the lab documents inside the lab.

QUT has an assembled SnakeRaven tool, if using the RAVEN II computer at QUT skip to step 3. Instructions, parts list, CAD files, videos and other resources to create your own SnakeRaven can be found in the appendix of my [thesis](https://eprints.qut.edu.au/235042/) and the bottom of this readme file

1. **Download** : On the RAVEN II computer ensure you have the packages snake_raven_controller/ and vision_system_snakeraven/ from this repository and place the folder in the RAVEN II catkin workspace folder e.g. home/raven_18_05

2. **Update** : Go to raven_18_05/raven_2 and update its contents with the three folders: /src /msg /include in the contents of folder [raven_2](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/tree/main/raven_2). Keep a back up of the original RAVEN II code just in case.

3. **Make** : Open a new terminal (ctrl-alt-t) and cd to the Raven II directory. Run catkin_make to compile the new content and modifications
Note: the sourcing step "source devel/setup.bash" is automatically being run on the QUT computer as it is being called in the .bashrc file
```
cd raven_18_05
source devel/setup.bash
catkin_make
```

4. **Run the Raven II** : If the SnakeRaven instrument isn't attached at this point wait until the calibration step. roslaunch the robot and a log message should indicate that you are running the modified version of the RAVEN II software. Press the e-stop, twist release and press the silver reset to go through homing. 
```
roslaunch raven_2 raven_2.launch
```
![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/raven_2_start.PNG)

5. **Velocity Joint Control mode** : Press 'm' to change mode and press '2' to start the velocity joint control mode, press the e-stop, twist release and press the silver reset for the mode change to occur.

6. **Run talkerSnakeRaven** : Open a new terminal and run command:
```
rosrun snake_raven_controller talkersnakeraven
```

7. **Run endoscopic camera node** : Open a new terminal and run command:
```
rosrun cv_camera cv_camera_node
```
Note: the endoscope is a USB camera connected to the computer and is device 0 at QUT. Depending on your system you may need to change the camera parameter to device number #:
```
rosparam set cv_camera/device_id #
```
Optionally, You can check the view of the image feed with command
```
rosrun image_view image_view image:=/cv_camera/image_raw
```
Optionally for a different device # like an external camera you can also rename the camera node but beaware this changes the topic name to subscribe to:
```
rosrun cv_camera cv_camera_node __name:=external _device_id=#
```

8. **Run endoscopic camera computer vision node** : Open a new terminal and run command:
```
rosrun vision_system_snakeraven imageprocessor
```

Layout of the four terminals for each of the ROS Nodes:

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/terminals.PNG)

Running rqt_graph in another terminal will visualise the ROS communication between these four nodes:

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/rqt_graph_diagram.png)

9. **Selection Menu** : All user interaction is conducted in the snake_raven_controller node

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/snakeraven_controller_menu.PNG)

_0. **Calibration** - this moves the arms to the required configuration given by the next user choice:

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/arm_menu.PNG)

Enter:
0. for right arm only, 1. for left arm only or 2. for dual arm configuration. 

In each of these cases the selected dominate arm or both arms are moved to be perpendicular to the table and the other arm is moved aside. It will pop up a message saying that you can insert the SnakeRaven tool onto the tool holder. 

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/calibration_screen.PNG)

You can use the keyboard to adjust the joints of the robot individually particularly to insert the SnakeRaven tool to mesh with the robot (see Joint Control section for keyboard mapping). 

This is also the ONLY point where you can re-tension the SnakeRaven tendons and use tweezers to place them back onto the pulley guides. DO NOT do this at any other stage or risk breaking the end-effector.

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/snakeraven_repair.PNG)

Good Calibration is when SnakeRaven is neutral and perpendicular to the table as seen in the SnakeRaven image below:

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/raven2_calibratedpose.PNG). 

Use the keyboard joint control to adjust the robot calibration manually. 

_1. **Joint Control** - this allows you to control the robot joints individually with the keyboard without calibration

**Joint Control Keyboard Mapping:**
See file [Keyboard_interactions.cpp](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/snake_raven_controller/src/Keyboard_interactions.cpp) 

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/jointmapping.PNG)

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

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/teleopmapping.PNG)

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

Additional keys in number pad for right arm use in dual arm setting:
- Z+     4
- Z-     6
- Y+     5
- Y-     2
- X+     3
- X-     1
- freeze z
- pan +  7
- pan -  9
- tilt + /
- tilt - 8
- z_rot+ -
- z_rot- *

_3. **Reset** - This moves both arms (or dominant arm) back to their perpendicular starting point as selected after calibration

_4. **Hand-eye calibration** - Warning: The outcome of this process is not very accurate and by default the hand-eye calibration is already set manually in the code. This mode only supports the right arm configuration assuming that is where the camera is attached.
1. Put an [aruco marker](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/vision_system_snakeraven/ArucoMarker1.png) in the camera field of view and keep it stationary
2. The program starts recording data points immediantely so the user must teleoperate the robot until the data collection is complete. After completion, the new camera to tool transform is displayed.

_5. **IBVS assisted Teleoperation** - Best Teleoperation demo but only supports the right arm. This allows you to control the SnakeRaven endeffector via keyboard relative to the camera view with the IBVS assist support (see how that works in my [thesis](https://eprints.qut.edu.au/235042/)) but only after calibration. It has some additional code in the Raven_Controller class to:
- record teleoperation control onto a .csv file in the home folder

**Endeffector Keyboard Mapping relative to camera view:**

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/blob/main/images/teleopmapping2.PNG)

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

## Additional Sources:
Check out my 
- [thesis](https://eprints.qut.edu.au/235042/) 
- [youtube for demo videos](https://www.youtube.com/channel/UCiLKwfcym1r7Ru3fDSA_D_A)
- [SnakeRaven assembly video](https://www.youtube.com/watch?v=k744cxB5OMc)
- [introduction to SnakeRaven video](https://www.youtube.com/watch?v=S8Rw0hFhcuw)
- Haptic devices at QUT instruction video
- How to use the [NDI electromagnetic Tracker](https://github.com/Andrew-Raz-ACRV/ndi_tracker_project/blob/main/QUT%20Northern%20Digital%20Inc%20Aurora%20Instructions.pdf) and my [code to it](https://github.com/Andrew-Raz-ACRV/ndi_tracker_project) for SnakeRaven
- [CAD files](https://grabcad.com/library/snakeraven-1)

And view the original QUT RAVEN II training resources from 2018:
- [How to use the RAVEN II training videos](https://www.youtube.com/playlist?list=PLxMsr-mRZng81BdDTaUX0sueXWeVOX0qd)
- The files in [raven_qut_training_docs_2018](https://github.com/Andrew-Raz-ACRV/SnakeRaven-Project/tree/main/raven_qut_training_docs_2018)
Note that this material can be out dated but has timeless resources such as the kinematics report, RAVEN history and CAD files

GitHub Code repositories to explore:
1. **uw-biorobotics/raven2** : [RAVEN II code repository](https://github.com/uw-biorobotics/raven2)
2. **AutoCircle_generator** : an example application of using ROS with the RAVEN II [AutoCircle_generator code](https://github.com/melodysu83/AutoCircle_generater)
3. My original works before this synthesis of the two packages: [**snake_raven_controller**](https://github.com/Andrew-Raz-ACRV/snake_raven_controller) and [**vision_system_snakeraven**](https://github.com/Andrew-Raz-ACRV/vision_servo_control_snakeraven)
4. MATLAB simulations: [snakeraven controller simulation](https://github.com/Andrew-Raz-ACRV/SnakeRavenSimulation) and [IBVS assisted teleoperation simulation](https://github.com/Andrew-Raz-ACRV/SnakeRaven_IBVS_simulation)

## Contact
This code is written by Andrew Razjigaev and describes the SnakeRaven system at QUT at the end of his PhD and work there from February 2023. If there are any queries about this project you can contact him via email: andrew_razjigaev@outlook.com
