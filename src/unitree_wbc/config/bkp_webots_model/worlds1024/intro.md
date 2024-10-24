# Backup for Webots Model
## Introduction
This folder contains the backup of the Webots model for the Unitree H1 robot. The model was created using the Webots simulation software and includes the robot's physical properties, sensors, and actuators. The model can be used to simulate the robot's basic behavior and test the WWBIC controller.

But it contains no detail scenes, so it is not suitable for the simulation of the robot's movement in the real world.

## Wrist Touch Sensor
Two touch sensor amounted the wrist of its arm, which has physis and mass, used for interaction force measurement.

At this stage, if we try to increase the mass or density of the wrist's touch sensor, the controller will crash.

##  Sole Touch Sensor
Also the foot sole touch sensor, which is used to detect the robot's contact with the ground. It has no effective influcement by now, because the robot don't test walk mode.