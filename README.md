# Prac-of-aviation-4-DOF-modeling
Practice and implementation aviation 4-DOF modeling in hanseo.univ

[Practice and implementation aviation 4-DOF modeling in hanseo.univ
notice "This is a basic model for 4DOF drone programming." "This simulation assumes there is no floor, meaning downward movement increases the negative altitude. Since this simulation is intended to quantify basic movements, attitude control is not applied when the rotor RPM increases."
Project Overview

# Drone PID Control Simulation

## Overview
This project implements a quadcopter altitude control system using a PID controller.

## Control Flow

Target Altitude
↓
PID Calculation
↓
RPM Generation
↓
Thrust and Torque Calculation
↓
Translational Motion Calculation
↓
Rotational Motion Calculation
↓
Attitude and Position Calculation
↓
Euler Integration
↓
State Update

## Main Functions

### calc_Thr_Tor()
Calculates thrust and torque from motor RPM values.

### calc_uvw_dot()
Calculates translational dynamics.

### calc_pqr_dot()
Calculates rotational dynamics.

### calc_att_pos_dot()
Calculates attitude and position changes.

### PID Controller
Adjusts motor RPM based on altitude error.

### Euler Integration
Updates system states.
## Example Code

```c
// PID output calculation
u_pid =
    Kp_D * err_D +
    Ki_D * err_D_int +
    Kd_D * err_D_dot;
```


## Simulation Result

![Altitude Graph](results/altitude_graph.png)
