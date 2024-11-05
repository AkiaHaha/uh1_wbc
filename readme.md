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

- The best for squating and ee moving with 5kg's box
```json
    "pelvisUpDown": -0.6,
    "pelvisForward": -0.08,
    "-----------------------":0,
    "pitchApt": 0.8,
    "motionFrq": 1,
    "armForward": 0.33,
    "armUpDown": -0.55,
    "armAside": 0.15,
    "wei EE Pos------------":0,
    "weightArmR": 1000,
    "weightArmP": 800,
    "weightArmY": 600,
    "weightArmX": 8000,
    "weightArmYY": 1000,
    "weightArmZ": 900,
    "weightFootX": 1200,
    "weightFootYY": 12000,
    "weightFootZ": 1200,
    "weightFootR": 1200,
    "weightFootP": 1200,
    "weightFootY": 20000,
    "ArmPD------------":0,
    "kpArmR": 1000.0,
    "kpArmP": 1000.0,
    "kpArmY": 1000.0,
    "kdArmR": 100.0,
    "kdArmP": 100.0,
    "kdArmY": 100.0,
    "kpArmX": 1200.0,
    "kpArmYY": 10000.0,
    "kpArmZ": 10000.0,
    "kdArmX": 120.0,
    "kdArmYY": 1200.0,
    "kdArmZ": 2000.0,