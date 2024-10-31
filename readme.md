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

# Branch Declaration
Test for the init pos of arm;

- Issue: When exe the task of squating while carrying the box, the hand will touch the knee so the motion will be broken; And then it couldn't put the box on the ground;

- Solution: The init pos of the arm is not good enough, try to open the arms so that when carrying heavy objects and dropping them, the wrist position is outside the knee range.

- Result: The stability is broken once starting the simulation, and didn't know the accurate reason.