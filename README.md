# SnakeRaven

A tendon-driven, 3D-printed snake-like continuum instrument for the RAVEN II surgical research robot, with the real-time control and vision software that drives it.

SnakeRaven mounts on the RAVEN II tool holder in place of a standard instrument. This repository is the fully integrated system as it stood at the end of my PhD: dual-arm teleoperation, an endoscopic vision system with hand-eye calibration, image-based visual servoing (IBVS) that assists the operator by holding the target in view, and autonomous waypoint navigation.

**Status:** archived. Completed March 2023 and not actively maintained. Built for ROS Kinetic against RAVEN II release 18_05. The methods are described in full in the papers and thesis below.

## Papers

The design, kinematics and control implemented here are published in:

> A. Razjigaev, A. K. Pandey, D. Howard, J. Roberts and L. Wu, "SnakeRaven: Teleoperation of a 3D Printed Snake-like Manipulator Integrated to the RAVEN II Surgical Robot," *2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2021, pp. 5282–5288. [doi:10.1109/IROS51168.2021.9636878](https://doi.org/10.1109/IROS51168.2021.9636878)

> A. Razjigaev, A. K. Pandey, D. Howard, J. Roberts, A. Jaiprakash, R. Crawford and L. Wu, "Optimal Vision-Based Orientation Steering Control for a 3-D Printed Dexterous Snake-Like Manipulator to Assist Teleoperation," *IEEE/ASME Transactions on Mechatronics*, vol. 29, no. 2, pp. 1260–1271, 2024. [doi:10.1109/TMECH.2023.3300662](https://doi.org/10.1109/TMECH.2023.3300662)

Full method and derivations: [PhD thesis](https://eprints.qut.edu.au/235042/).

<!-- Citations verified against profile/master-cv-academic.md lines 207 and 219,
     which github/CLAUDE.md designates the authority for publications.

     DRIFT (flagged, not resolved): master-cv-academic.md dates the T-Mech paper 2024
     (vol 29 no 2, the issue date), while profile/hardware-coding/snakeraven-platform.md
     cites it as 2023 (the online-first date, matching the DOI's TMECH.2023 prefix).
     Both are defensible; they should not disagree. The academic CV wins here per
     CLAUDE.md's source-of-truth table, but profile/ should be made consistent.
     Every github/ file written before 2026-09-07 said "T-Mech 2023" and has been
     corrected. NEEDS ANDREW: which does he want as the canonical year? -->

## See it working

- [Introduction to SnakeRaven](https://www.youtube.com/watch?v=S8Rw0hFhcuw)
- [Assembly walkthrough](https://www.youtube.com/watch?v=k744cxB5OMc)
- [Demo videos](https://www.youtube.com/channel/UCiLKwfcym1r7Ru3fDSA_D_A)
- [CAD files on GrabCAD](https://grabcad.com/library/snakeraven-1)

Parts list, CAD and build instructions for making your own SnakeRaven are in the appendix of the thesis.

## What is in here

| Package | What it does |
| --- | --- |
| `snake_raven_controller` | Kinematics and control. Forward/inverse kinematics for the multi-module continuum section, teleoperation modes, waypoint tasks, keyboard interaction. |
| `vision_system_snakeraven` | Endoscopic camera processing, ArUco detection, hand-eye calibration, and the IBVS control action. |
| `raven_2` | Modified files for the RAVEN II control software. Replaces `/src`, `/msg`, `/include` in an existing RAVEN II install. |
| `raven_qut_training_docs_2018` | Original QUT RAVEN II training material — kinematics report, history, CAD. Dated but the kinematics report is still useful. |

This repository supersedes two earlier packages of mine, [`snake_raven_controller`](https://github.com/Andrew-Raz-ACRV/snake_raven_controller) and [`vision_servo_control_snakeraven`](https://github.com/Andrew-Raz-ACRV/vision_servo_control_snakeraven) — it is the synthesis of the two. **Start here rather than there.**

MATLAB simulations of the same methods, which run without any hardware: [controller simulation](https://github.com/Andrew-Raz-ACRV/SnakeRavenSimulation) and [IBVS teleoperation simulation](https://github.com/Andrew-Raz-ACRV/SnakeRaven_IBVS_simulation).

Electromagnetic tracking used to validate the kinematics: [`ndi_tracker_project`](https://github.com/Andrew-Raz-ACRV/ndi_tracker_project).

## What runs without a RAVEN II

Very little of this repository does. The control and vision nodes assume a RAVEN II and a physical SnakeRaven instrument, and there is no simulation backend here.

**If you do not have the hardware, the two MATLAB simulators linked above are what you want** — they implement the same kinematics and IBVS method and run standalone.

## Installing

### Dependencies

- ROS Kinetic. This was not developed or tested against later distributions.
- [RAVEN II control software](https://github.com/uw-biorobotics/raven2), release 18_05.
- [Eigen](https://eigen.tuxfamily.org) — header-only, and not vendored here.
- `cv_camera` for the USB endoscope.

### Eigen

Download Eigen, then copy the `Eigen` and `unsupported` subfolders into an `include/` folder inside both `snake_raven_controller/` and `vision_system_snakeraven/`.

### Building

Place both packages in your RAVEN II catkin workspace, then replace the contents of `raven_18_05/raven_2` with the `/src`, `/msg` and `/include` folders from this repository's `raven_2`. **Keep a backup of the original RAVEN II code.**

```bash
cd raven_18_05
source devel/setup.bash
catkin_make
```

## Running

Four nodes, one terminal each.

```bash
# 1. the robot
roslaunch raven_2 raven_2.launch

# 2. the controller (all user interaction happens here)
rosrun snake_raven_controller talkersnakeraven

# 3. the endoscope
rosrun cv_camera cv_camera_node

# 4. vision processing
rosrun vision_system_snakeraven imageprocessor
```

After launching the robot, press the e-stop, twist to release, and press the silver reset to home. Then press `m` and `2` to enter velocity joint control mode, and repeat the e-stop/release/reset for the mode change to take effect.

If your endoscope is not device 0: `rosparam set cv_camera/device_id <n>`. To check the feed: `rosrun image_view image_view image:=/cv_camera/image_raw`.

`rqt_graph` will show the four nodes and the topics between them.

## Modes

All interaction is through the `snake_raven_controller` menu.

| Mode | What it does |
| --- | --- |
| **0 — Calibration** | Moves the selected arm (right, left, or dual) perpendicular to the table so the SnakeRaven tool can be fitted. Use joint control to fine-tune the mesh. |
| **1 — Joint control** | Per-joint keyboard control, no calibration required. |
| **2 — Teleoperation** | End-effector keyboard control in the robot frame. Logs to CSV in the home folder. |
| **3 — Reset** | Returns the arms to the post-calibration starting pose. |
| **4 — Hand-eye calibration** | Estimates the camera-to-tool transform from an ArUco marker. Right arm only. |
| **5 — IBVS-assisted teleoperation** | End-effector control *relative to the camera view*, with the visual-servoing assist holding the target in frame. Right arm only. The best demonstration of the system. |
| **6 — Waypoint navigation** | The only fully autonomous mode. Traces a defined set of waypoints. Works in any arm configuration. |

**Two warnings that matter.**

**Re-tension the tendons only during calibration.** That is the one point where the tool can be safely adjusted and the tendons placed back on the pulley guides with tweezers. Doing it at any other stage risks breaking the end-effector.

**Hand-eye calibration (mode 4) is not very accurate**, and a manually determined transform is already set in the code. Use mode 4 only if you have reason to re-estimate it.

### Keyboard maps

Defined in [`Keyboard_interactions.cpp`](snake_raven_controller/src/Keyboard_interactions.cpp).

**Joint control** — left arm `1/q` `2/w` `3/e` `4/r` `5/t` `6/y` `7/u`, right arm `a/z` `s/x` `d/c` `f/v` `g/b` `h/n` `j/m`, in the order shoulder, elbow, Z insertion, tool rotation, wrist, grasp 1, grasp 2.

**Teleoperation** — `w`/`s` forward/retreat, `q`/`e` up/down, `a`/`d` left/right, `z` freeze, `t`/`g` bend up/down, `f`/`h` bend left/right. In IBVS mode, `1` toggles the assist. The number pad drives the right arm in dual-arm mode.

## Running it on the QUT RAVEN II

<!-- Site-specific. Kept because it is genuinely useful to the next person in that
     lab and exists nowhere else, but it is not general instruction. -->

The QUT machine has an assembled SnakeRaven tool, so you can skip straight to building.

On the bottom stack, turn on the 48 V power and wait five seconds. Turn on the system power, then the power button on the fourth stack to start the computer. Login details are in the lab documentation. `source devel/setup.bash` is already in `.bashrc` there.

Historical RAVEN II training material for that lab: [training videos](https://www.youtube.com/playlist?list=PLxMsr-mRZng81BdDTaUX0sueXWeVOX0qd) and the `raven_qut_training_docs_2018` folder. Also: [haptic devices at QUT](https://youtu.be/e6HqHnoaPHQ).

## Licence

MIT — see [LICENSE](LICENSE). The same licence as the [RAVEN II software](https://github.com/uw-biorobotics/raven2) this builds on.

<!-- RESOLVED 2026-09-07 (Andrew): MIT, chosen to match RAVEN II. That is the correct
     call — MIT is permissive and compatible, so redistributing modified RAVEN II
     files under MIT raises no conflict.

     The gap was never the licence, it was that the README never mentioned it. A
     visitor deciding whether they can build on this reads the README, not the
     sidebar. One line fixes it.

     ONE COMPLIANCE POINT WORTH CHECKING: MIT requires the original copyright notice
     to travel with substantial portions of the software. The raven_2/ folder here
     redistributes modified upstream files, so it should carry uw-biorobotics'
     copyright notice alongside Andrew's — not only his. Worth a look at what is
     currently in that folder. Low stakes, easy to fix, and the kind of thing a
     careful reader notices. -->

<!-- GAP: does QUT hold any claim over PhD-produced code? MIT is already published so
     this is likely settled in practice, but worth knowing. -->

## Citing

If you use this work, cite the IROS 2021 paper for the platform and kinematics, or the T-Mech 2023 paper for the vision-based steering control.

<!-- Add a CITATION.cff at the repo root so GitHub renders a "Cite this repository"
     button. Costs nothing, makes citation the path of least resistance. -->

## Questions

Written by Andrew Razjigaev. This describes the SnakeRaven system at QUT as it stood at the end of my PhD, February–March 2023. Questions: andrew_razjigaev@outlook.com
