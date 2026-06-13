# Prac-of-aviation-4-DOF-modeling
Practice and implementation aviation 4-DOF modeling in hanseo.univ

[Practice and implementation aviation 4-DOF modeling in hanseo.univ
notice "This is a basic model for 4DOF drone programming." "This simulation assumes there is no floor, meaning downward movement increases the negative altitude. Since this simulation is intended to quantify basic movements, attitude control is not applied when the rotor RPM increases."
Project Overview

This is a basic model for 4DOF drone programming.
The simulation operates under the assumption that there is no floor; thus, downward movement results in an increasingly negative altitude. Furthermore, because the primary objective of this simulation is to quantify fundamental movements, attitude control is disabled during periods of increased rotor RPM.
Target Altitude ↓ PID Controller ↓ Motor RPM Command ↓ Thrust & Torque Calculation ↓ Translational Motion ↓ Rotational Motion ↓ Attitude & Position Calculation ↓ Euler Integration ↓ State Update
Main Functions calc_Thr_Tor()
Calculates thrust and torque from motor RPM values.
calc_uvw_dot()
Calculates translational dynamics (linear motion).
u_dot : Velocity acceleration along the x-axis v_dot : Velocity acceleration along the y-axis w_dot : Velocity acceleration along the z-axis calc_pqr_dot()
Calculates rotational dynamics (angular motion).
p_dot : Roll rate acceleration q_dot : Pitch rate acceleration r_dot : Yaw rate acceleration calc_att_pos_dot()
Calculates attitude and position changes.
Roll (phi) Pitch (theta) Yaw (psi) Position (P_N, P_E, P_D) PID Controller
Adjusts motor RPM based on altitude error.
Proportional (P) Control Integral (I) Control Derivative (D) Control Euler Integration
Updates system states using numerical integration.
Velocity Update Angular Velocity Update Attitude Update Position Update

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
