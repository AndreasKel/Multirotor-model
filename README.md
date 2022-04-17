# Multirotor-model
A Matlab/Simulink model of a multirotor
## Overview

Multirotor is an unmanned aerial vehicle with propulsors that are enabling vertical takeoff and landing. Direct influence on the multirotor movement is induced by variation of each motor’s RPM.

## Mathematical Model
Mathematical model describes multirotor movement and behaviour with the respect to the input values of the model and external influences on 
multirotor to predict position and attitude of multirotor by knowing the angular 
velocities of propellers.

Multirotor dynamics are described by differential equations that were derived by using the Newton Euler method. Dynamics of a 6 DOF body takes into consideration the mass and the inertia of the body. The number of controllable variables is equal to the number of propulsors which affect position/velocity and attitude of multirotor in space.
### Assumptions
- Frame is rigid. 
- Frame is symmetric. 

---
__Simulink Model__
![Simulink screenshot](https://raw.githubusercontent.com/AndreasKel/Multirotor-model/main/SimulinkScreenshot.PNG?raw=true)