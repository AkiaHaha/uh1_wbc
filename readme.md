This is the wbc controller for Unitree H1 robot;
Worked in ROS, GNU C+++, Eigen, QPOASES, RBDL;

# Install  External Library
    - QPOASES
    - Eigen
    - RBDL

# Install gdb for debug
- sudo apt install gdb

# The Params
- nG
- nJ
- nFc
- nV

# The Variables
- The final optimization variable
xOpt [6(Floating Base Degree), 19(Joints), 6(Left Foot Intact Wrench{Torque, Force}), 
      6(Right Foot Intact Wrench{Torque, Force}), 6(Left Wrist Contact Wrench{Torque, Force}), 6(Right Wrist Contact Wrench{Torque, Force})]


# The Functions


# The Naming Method
- wrist
- foot
- arm
- sole
- torso
- pelvis
- shoulder
- elbow
- hip
- knee
- ankle
- contact
- wrench
- twist
- torque
- force

# Drafts
- The Bkp
```json
    "pelvisUpDown": -0.65,
    "pelvisForward": -0.05,
    "-----------------------":0,
    "pitchApt": 0.8,
    "motionFrq": 1,
    "armForward": 0.35,
    "armUpDown": -0.65,

- The best for now
```json
    "pelvisUpDown": -0.65,
    "pelvisForward": -0.05,
    "-----------------------":0,
    "pitchApt": 0.8,
    "motionFrq": 1,
    "armForward": 0.25,
    "armUpDown": -0.82,
    "armAside": 0.0,

- Set to zero
```json
    "pelvisUpDown": -0.0,
    "pelvisForward": -0.0,
    "-----------------------":0,
    "pitchApt": 0.0,
    "motionFrq": 1,
    "armForward": 0.0,
    "armUpDown": -0.0,
    "armAside": 0.0,

- The best for squating and ee moving
```json
    "pelvisUpDown": -0.55,
    "pelvisForward": -0.08,
    "-----------------------":0,
    "pitchApt": 0.8,
    "motionFrq": 1,
    "armForward": 0.33,
    "armUpDown": -0.82,
    "armAside": 0.1,

- The best for squating and ee moving with 10kg's box
```json
    "pelvisUpDown": -0.6,
    "pelvisForward": -0.08,
    "pelvisAside": -0.0,
    "-----------------------":0,
    "motionFrq": 1.0,
    "sFreq": 1.0,
    "pitchApt": 0.8,
    "rollApt": 0.0,
    "yawApt": 0.0,
    "armForward_L": 0.33,
    "armForward_R": 0.33,
    "armUpDown_L": -0.55,
    "armUpDown_R": -0.55,
    "armAside_L": 0.15,
    "armAside_R": -0.15,
    "armRoll_L": -0.0,
    "armRoll_R": -0.0,
    "armPitch_L": 0.0,
    "armPitch_R": 0.0,
    "armYaw_L": 0.0,
    "armYaw_R": 0.0,
    "footForward": 0.0,
    "footAside": 0.0,
    "footUpDown": 0.0,
    "footRoll": 0.0,
    "footPitch": 0.0,
    "footYaw": 0.0,