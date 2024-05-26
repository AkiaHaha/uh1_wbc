# TAICHI使用说明

TAICHI是基于二次规划(QP)的任务处理和求解工具，所以我们最终会将任务的求解写为QP的形式

<img src="https://latex.codecogs.com/gif.latex?\min_{\mathbf{x}}||\mathbf{A}\mathbf{x}-\mathbf{b}||_2^2"/>

<img src="https://latex.codecogs.com/gif.latex?\mathbf{Cx}\leq \mathbf{ub}"/>

为了构建这个优化问题实现任务求解，我们需要有四个步骤：

- 描述系统动力学
- 描述控制任务为<img src="https://latex.codecogs.com/gif.latex?\mathbf{Ax}=\mathbf{b}"/>的形式
- 描述约束为<img src="https://latex.codecogs.com/gif.latex?\mathbf{Cx}\leq \mathbf{ub}"/>的形式
- 控制中实时给出任务期望，求解QP得到<img src="https://latex.codecogs.com/gif.latex?\mathbf{x}"/>

下面用biped例子进行说明。

### 描述系统动力学

首先基于robotDynamics进行动力学建模，双足机器人的动力学在‘robotDynamicsBiped.cpp’中按rbdl的方式进行了建模，用户也可尝试用rbdl直接读取urdf模型。然后主要调用rbdl进行一些雅可比矩阵
<img src="https://latex.codecogs.com/gif.latex?\mathbf{J}"/>和
<img src="https://latex.codecogs.com/gif.latex?\dot{\mathbf{J}}\dot{\mathbf{q}}"/>的计算
<img src="https://latex.codecogs.com/gif.latex?\mathbf{q}"/>为机器人广义坐标，包含浮动基坐标和关节角)，用于后续相应控制任务的描述，例如biped里计算了

- 足末端<img src="https://latex.codecogs.com/gif.latex?\mathbf{J}_{foot}"/>和<img src="https://latex.codecogs.com/gif.latex?\dot{\mathbf{J}}_{foot}\dot{\mathbf{q}}"/>: dualSoleJacob, dualSoleJDotQDot
- 躯干<img src="https://latex.codecogs.com/gif.latex?\mathbf{J}_{waist}"/>和<img src="https://latex.codecogs.com/gif.latex?\dot{\mathbf{J}}_{waist}\dot{\mathbf{q}}"/>: waistJacob, waistJDotQDot
- 重心动量<img src="https://latex.codecogs.com/gif.latex?\mathbf{J}_{com}"/>和<img src="https://latex.codecogs.com/gif.latex?\dot{\mathbf{J}}_{com}\dot{\mathbf{q}}"/>: centroidAG, centroidAGDotQDot

后续会用于足末端任务、躯干任务和质心任务的描述和控制中。

另外在robotDynamics中也可调用rbdl进行一些运动学计算，方便后续控制中状态估计时的调用，例如‘robotDynamicsBiped.cpp’中通过正运动学计算了

- 躯干位置速度: 函数estWaistPosVelInWorld
- 摆动足位置速度: 函数estFootPosVelInWorld

### 描述控制任务

对于任何一个控制对象<img src="https://latex.codecogs.com/gif.latex?\mathbf{x}_{task}"/>(足位置、躯干位置、质心位置等)，其加速度为: 
<img src="https://latex.codecogs.com/gif.latex?\ddot{\mathbf{x}}_{task} = \mathbf{J}\ddot{\mathbf{q}} + \dot{\mathbf{J}}\dot{\mathbf{q}}"/>

若期望加速度为<img src="https://latex.codecogs.com/gif.latex?\ddot{\mathbf{x}}_{task}^{ref}"/>, 则控制目标即为: 
<img src="https://latex.codecogs.com/gif.latex?\ddot{\mathbf{x}}_{task}^{ref} = \mathbf{J}\ddot{\mathbf{q}} + \dot{\mathbf{J}}\dot{\mathbf{q}}\Rightarrow \mathbf{J}\ddot{\mathbf{q}} = \ddot{\mathbf{x}}_{task}^{ref} - \dot{\mathbf{J}}\dot{\mathbf{q}}"/>

这样我们就可以将控制任务描述为<img src="https://latex.codecogs.com/gif.latex?\mathbf{A}\mathbf{x}=\mathbf{b}"/>的形式，其中优化变量
<img src="https://latex.codecogs.com/gif.latex?\mathbf{x}"/>为<img src="https://latex.codecogs.com/gif.latex?\ddot{\mathbf{q}}"/>, 
<img src="https://latex.codecogs.com/gif.latex?\mathbf{A}"/>为<img src="https://latex.codecogs.com/gif.latex?\mathbf{J}"/>, <img src="https://latex.codecogs.com/gif.latex?\mathbf{b}"/>为
<img src="https://latex.codecogs.com/gif.latex?\ddot{\mathbf{x}}_{task}^{ref}-\dot{\mathbf{J}}\dot{\mathbf{q}}"/>(均为给定值或当前可知值)。

实际上，在biped例子中使用的优化变量为<img src="https://latex.codecogs.com/gif.latex?\ddot{\mathbf{q}}"/>和足底力
<img src="https://latex.codecogs.com/gif.latex?\mathbf{f}"/>, 只需对
<img src="https://latex.codecogs.com/gif.latex?\mathbf{A}"/>相应扩充修改。

使用这种方法在‘taskDefinitionBiped.cpp’里进行了多种任务的描述，包括：

- 质心动量任务: BipedCentroidalMomentum
- 躯干位置姿态任务: BipedTorsoPosition
- 足端位置姿态任务: BipedFootPosition
- 足底力任务: BipedFootForce
- 足底力变化任务: BipedFootForceChange

### 描述约束

在’constraintDefinitionBiped.cpp‘里将不等式约束描述为<img src="https://latex.codecogs.com/gif.latex?\mathbf{Cx}\leq \mathbf{ub}"/>的形式。以足底力摩擦锥为例，

<img src="https://latex.codecogs.com/gif.latex?|f_x|\leq&space;\mu&space;f_z&space;\Rightarrow&space;-\infty&space;<&space;\begin{bmatrix}&space;1&space;&&space;-\mu&space;\\&space;-1&space;&&space;\mu&space;\end{bmatrix}&space;\begin{bmatrix}&space;f_x\\&space;f_z&space;\end{bmatrix}&space;\leq&space;0."/>

类似的，biped例子里提供了多种约束的描述，包括：

- 系统动力学约束: BipedDynamicConsistency
- 足底力摩擦锥约束: BipedFrictionCone
- 足底压力中心约束: BipedCenterOfPressure
- 关节力矩约束: BipedJointTorqueSaturation

### 任务求解

任务的求解需要使用TAICHI中提供的wqpWbc或hqpWbc求解器来调用前述构建的动力学、任务和约束来进行求解。Biped例子中在‘bipedController.cpp’中使用wqpWbc进行任务的求解。

首先在BipedController的构建函数BipedController::BipedController()中，使用addTask, addConstraint添加控制任务和约束然后来实例化一个wqpWbc求解器。然后具体每个控制周期执行的函数为BipedController::update，其中包含的主要步骤为状态估计(stateEstimation)，任务规划(motionPlan)以及控制任务求解(taskControl)。状态估计和任务规划函数需要用户自己根据需求构建，在biped例子中，stateEstimation中调用robotDynamicsBiped进行了躯干位姿和其速度，摆动脚的位姿和其速度的反馈，motionPlan中给了简单的躯干高度、摆动脚高度和姿态的正余弦轨迹规划。

在控制任务求解函数BipedController::taskControl中，需要根据stateEstimation得到的任务状态反馈
<img src="https://latex.codecogs.com/gif.latex?\mathbf{x}_{task}^{est},\dot{\mathbf{x}}_{task}^{est}"/>和motionPlan得到的任务规划
<img src="https://latex.codecogs.com/gif.latex?\mathbf{x}_{task}^{d},\dot{\mathbf{x}}_{task}^{d}"/>来计算给wqpWbc的期望任务加速度
<img src="https://latex.codecogs.com/gif.latex?\ddot{\mathbf{x}}_{task}^{ref}"/>, biped例子里使用的是经典的PD控制方法，即

<img src="https://latex.codecogs.com/gif.latex?\ddot{x}_{task}^{ref}&space;=&space;k_p(x_{task}^{d}&space;-&space;x_{task}^{est})&space;&plus;&space;k_d(\dot{x}_{task}^{d}&space;-&space;\dot{x}_{task}^{est})"/>

然后求解QP并转化为关节力矩tauOpt.

在biped例子的主函数main中每个周期调用BipedController::update然后发送力矩给webots中的机器人，完成的单腿站立蹲起摆腿的控制。
