# RealManRobot_control

## running project after *"building steps"*

### run drivers

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch RealManRobot_control robot_bringup.launch 
```

### run music demo
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch RealManRobot_control music_bringup.launch 
```

# Project tree
```bash
├── CMakeLists.txt
├── include
│   └── RealManRobot_control
├── launch
│   ├── music_bringup.launch #[ROS] launches movement publishers
│   └── robot_bringup.launch #[ROS] launches robot subscribers
├── package.xml
├── README.md
├── scripts
│   ├── audio #folder which must contains audio in .wav for playing music in realtime with robot movement
│   │   └── .wav
│   ├── head_control
│   │   ├── head_sub.py #[ROS] head subscriber
│   │   ├── lewansoul_servo_bus.py #servo drivers
│   │   └── __pycache__
│   │       └── lewansoul_servo_bus.cpython-38.pyc #servo drivers
│   ├── lips_control
│   │   ├── generate_servo_setup
│   │   │   ├── audio #folder for audio in .wav to prepare lips servo angles in timestamps
│   │   │   │   ├── README.md
│   │   │   │   └── .wav
│   │   │   ├── generate_by_audio.py #file for generating lips angle in timestamp from audio
│   │   │   ├── generate_by_text_in_audio.py #not tested yet
│   │   │   └── generated
│   │   │       ├── servo_angles.txt #generated angles
│   │   │       ├── servo_movements.txt #generated angles in timestamps
│   │   │       └── timestamps.txt #generate timestamps
│   │   └── move_lips
│   │       ├── move_lips_servos.py #simple audio test
│   │       └── move_lips_terminal_demo.py #simple audio test with music
│   └── robot_music_pub.py #[ROS] runs music and movement to it
└── src

13 directories, 19 files

```

# Notes of package

## folder ```/launch``` - it runs all project

### robot_bringup.launch

- launch head subscrber
    - from path `/scripts/head_control/head_sub.py`

- launch arm subscriber
    - from other path `arm_driver/launch/dual_arm_65_driver.launch`

### music_bringup.launch

- launch movement publisher
    - from path `/scripts/robot_music_pub.py`

# main files description

## `scripts/head_control/head_sub.py`
contain subscribers for only (open close eyes, open close lips)

## `/scripts/robot_music_pub.py`

- contain launching lips movement from angles data
```python
    if angle > 80:
        mouthCloseAndOpen(True)
        rospy.loginfo(f"Czas: {t:.2f} s, singing{angle:.2f}")
    rospy.loginfo(f"Czas: {t:.2f} s, Wykonano ruch: {angle:.2f}")
```
- contain arm movement in choosed timestamp
```python
robot_actions = {
    0.0: [(mata_hand_left, (500,)), (mata_hand_right, (500,))],
    5.0: [(hand_left_close, (100,)), (hand_right_close, (900,))],
    6.0: [(hand_left_open, (100,)), (hand_right_open, (900,))],
    7.0: [(hand_left_close, (100,)), (hand_right_close, (900,))],
    8.0: [(mata_hand_left, (500,)), (mata_hand_right, (500,)), (setEyelidsBlink, (False,)), (mouthCloseAndOpen, (True,))],
    13.0: [(left_arm, ()), (right_arm, ())],
    18.0: [(left_arm2, ()), (right_arm2, ())],
    20.0: [(left_arm_mata, ()), (right_arm_mata, ()), (setEyelidsBlink, (True,)), (mouthCloseAndOpen, (False,))]
}
```

[More instruction in this link](https://develop.realman-robotics.com/en/robot/ros/getStarted/)