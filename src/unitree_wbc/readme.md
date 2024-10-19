clee# AGIROBOT : Task Arrangement In Control HIerarchy

AGIROBOT is a robot whole-body control toolbox, originated from an internal project of Beijing Research Institute of UBTECH Robotics. It aims to arrange and solve tasks with changing control hierarchy automatically, which allows users to modify the control hierarchy freely online according to their needs. AGIROBOT provides interfaces to define robot dynamics, tasks and constraints in the form of abstract classes. Quadratic Programming (QP) based whole-body control solvers are provided for task solving.

To illustrate the usage of AGIROBOT, a 12 DoF biped example is provided, along with its task and constraint libraries. Users can directly use these libraries to control similar robots. 
Detailed guide to this toolbox can be seen in the tutorial [biped_example](http://10.10.1.70/AGIROBOT/AGIROBOT/wikis/AGIROBOT:-Biped-Example).
## Features
In the current version of TACHI, two types of whole-body control solvers are provided:
- WQP: Weighted Quadratic Programming, where tasks are solved in one QP, weighting coefficients are used to adjust the priority of tasks.
- HQP: Hierarchical Quadratic Programming, where tasks are placed in different hierarchy levels and solved with a sequence of QPs, tasks in the lower level won't influence the tasks in the higher level.

In the future release, two more features will be added
- RHP-HQP Solver[1]: HQP solver with Recursive Hierarchical Projection, which can realize the smooth transition of task hierarchy and weight. The current WQP and HQP solver will be unified in this solver.
- MPC Task Controller[2]: The task acceleration references are given by the task-level MPC, which can handle inequality contraints better than traditional whole-body controllers based on PD task control.

## Installation
**System Requirements:** Ubuntu 16.04 LTS, Ubuntu 18.04 LTS, Ubuntu 20.04 LTS

**Compiling Environments:**  cmake >= 3.0, gcc >= 5.0, g++ >= 5.0, C++ compiler with C++11 support

**Dependent Libraries:** [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)[5], [qpOASES](https://github.com/coin-or/qpOASES)[6], [RBDL](https://github.com/rbdl/rbdl)[7]. Refer to correspoding websites for installation

**Optional Dependencies:** [Doxygen](https://www.doxygen.nl/index.html), for generating documentation from source code; [Webots](https://cyberbotics.com/) >= R2021b, for simulation/visualization of the example "Biped"; Refer to correspoding websites for installation

**CMake Options:**

  1.  *USE_ERROR* : The Error Message displays as a warning (OFF) or catch it yourself (ON), default 'ON'
  2.  *COMPILE_EXAMPLE* : Compile the biped example (ON) or not (OFF), default 'ON'

**Compile and install AGIROBOT:**

- Enter the project directory, make build directory, cmake, compile, install STATIC library

```shell
mkdir build
cd build
cmake ..
make
make install
```

This will generate a sub-folder 'AGIROBOT' in the 'build' folder, it contains the header files and static libraries for AGIROBOT, users can copy this folder to their project directory.

**Generate the documentation：**

- Enter the project directory and run

```shell
doxygen Doxyfile
```

This will generate a 'doc' folder with a html documentation. 

## Example
A 12 DoF biped example is provided, where the biped is controlled to stand on one leg. The torso height, swing foot height and its roll, pitch and yaw angles are controlled to follow sinusoidal trajectories. Detailed 
guide to this toolbox can be seen in the tutorial [biped_example](http://10.10.1.70/AGIROBOT/AGIROBOT/wikis/AGIROBOT:-Biped-Example).

**Run the example:**

1. Set *COMPILE_EXAMPLE* as ON in the CmakeLists.txt

2. Enter the 'build' directory (create one, if it does not exist), cmake, compile
	```shell
    cd build
    cmake ..
    make
   ```
   
3. Open the world 'Biped.wbt' in Webots and run, it is located in "AGIROBOT/example/Biped/WebotsProject/worlds" (Note: if your webots version is under R2021b, you may need to change the gravity direction to 'z' in WorldInfo of 'Biped.wbt')

<img src="http://10.10.1.70/AGIROBOT/AGIROBOT/raw/gif/biped.gif"/>

## Contributors

王家俊，鞠笑竹，郭宜劼，韩刚，谢岩

## Related Publications

1. Gang Han, Jiajun Wang, Xiaozhu Ju, and Mingguo Zhao. Recursive Hierarchical Projection for Whole-Body Control with Task Priority Transition. arXiv preprint arXiv:2109.07236, 2021.
2. Xiaozhu Ju, Jiajun Wang, Gang Han, and Mingguo Zhao. Mixed Control for Whole-Body Compliance of a Humanoid Robot. arXiv preprint arXiv:2109.07705, 2021.
3. Jiajun Wang, Gang Han, Xiaozhu Ju, and Mingguo Zhao. Whole-Body Control with Motion/Force Transmissibility for Parallel-Legged Robot. arXiv preprint arXiv:2109.07196, 2021.
4. Yan Xie, Jiajun Wang, Hao Dong, Xiaoyu Ren, Liqun Huang, and Mingguo Zhao. Dynamic Balancing of Humanoid Robot Walker3 with Proprioceptive Actuation: Systematic Design of Algorithm, Software and Hardware. arXiv preprint arXiv:2108.03826, 2021.
5. Gael Guennebaud, Benoit Jacob and others. Eigen v3, http://eigen.tuxfamily.org, 2010. 
6. Hans Joachim Ferreau,  Christian Kirches,  Andreas Potschka,  Hans Georg Bock,  and  Moritz Diehl. qpOASES:  A  parametric  active-set  algorithm  for  quadratic  programming.  Mathematical  Programming  Computation, vol. 6, no. 4, pp.327–363, 2014.
7. Martin L. Felis. RBDL: an efficient rigid-body dynamics library using recursive algorithms. Autonomous Robots, vol. 41, no. 2, pp. 495–511, 2017.

