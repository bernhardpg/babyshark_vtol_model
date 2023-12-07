# Dynamic Model of the Foxtech Babyshark 260 VTOL UAV
Full dynamic model for the Babyshark 260 VTOL UAV.

<img src="https://user-images.githubusercontent.com/35483844/139727366-eb39f3f5-01c7-4e4c-86a9-0f0902c28979.png" width="800">

**If you use this model, please cite the original thesis:**
> B. P. Graesdal, “Full Nonlinear System Identification for a Vertical-Takeoff-and-Landing Unmanned Aerial Vehicle,” Master thesis, NTNU, 2021. Available: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2981320

The repo contains three main files:
1. *model/BabysharkModel.m*: Aircraft model.
2. *visualizer/AircraftVisualizer.m*: Simple 3D visualization tool for the babyshark model.
3. *3d_files/babyshark.stl*: 3D model of the Babyshark created for this work.

The model is written entirely in Matlab. However, porting the model to any other language should be straightforward. All of the required equations and parameters can be found in *BabysharkModel.m*.

## About the model
The model simulates the 12 dimensional rigid-body equations of motion that
govern the aircraft motion. The nonlinear aerodynamic model is
derived from flight-test data around trim conditions.
In addition, the model simulates the control surface dynamics for the
three control surfaces delta_a, delta_e, delta_r.

The model is validated and performs well for flight close to trim conditions (`V (airspeed) = 21 m/s`).

### State and input
* **State**: `[n, e, d, u, v, w, p, q, r, phi, theta, psi, delta_a, delta_e, delta_r]`
* **Input**: `[delta_a_sp delta_e_sp delta_r_sp delta_t delta_mr_1 delta_mr_2 delta_mr3 delta_mr_4]`

### Actuators
* `delta_a`, `delta_e`, `delta_r` denote the actual control surface
deflections in radians. These are limited by maximum deflection angles.
* `delta_a_sp`, `delta_e_sp`, `delta_r_sp` denote their respective setpoints
in radians.
* `delta_t` denotes the squared RPS (rev/second) for the fixed-wing propeller.
* `delta_mr_i, i = [1,4]`, denotes the squared RPS (rev/second) for the multirotor propellers.

The control surfaces are defined with the following sign conventions:
<img src="https://user-images.githubusercontent.com/35483844/139251373-e6deee3a-46ed-4fdf-b736-263ed808e8fa.jpg" width="600">

The multirotor propellers are defined in the following order:
<img src="https://user-images.githubusercontent.com/35483844/139251337-1b7a43ef-1670-4dc9-8ce9-1fa7a64ec2cd.jpg" width="600">

## Running the examples
There are two examples included in the repo:
1. *simulate_feedback_controller.m*: Example file that simulates the model with a simple feedback controller and visualizes the simulated response.
2. *simulate_recorded_input.m*: Example file that loads recorded inputs, simulates the model and visualizes the aircraft response.

Both of the examples should run out of the box. Note: Remember to add all files in the repo to path in Matlab before running!

## Caveats
* For small airspeeds (V < 1), the AoA and SSA will are set
equal to 0 to avoid numerical problems in this flight regime.
* The aircraft model is developed from flight-test-data around trim conditions. For flight regimes close to stall, the model is not expected to predict the lift and drag accurately. For use in this flight regime, future work on the model is needed.
* The model does not implement constraints on the maximum multirotor or the fixed-wing propeller speed.

## More information
This aircraft model is the result of my master's thesis, which can be found in the file *documentation/full_nonlinear_system_identification_for_a_vtol_uav.pdf*. The thesis includes detailed information on the model, the modelling approach, model performance, and future work. The source code for the entire system identification procedure can be found at: [source code](https://github.com/bernhardpg/vtol-system-identification).

