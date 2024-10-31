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
    "pelvisUpDown": -0.65,
    "pelvisForward": -0.05,
    "-----------------------":0,
    "pitchApt": 0.8,
    "motionFrq": 1,
    "armForward": 0.35,
    "armUpDown": -0.65,