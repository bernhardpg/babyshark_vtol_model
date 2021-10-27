clc; close all; clear all;

% Read example maneuver
recorded_input = readmatrix("example_inputs/yaw_maneuver_left_input.csv");
dt = readmatrix("example_inputs/dt.csv");
t_end = length(recorded_input) * dt - dt;
tspan = [0 t_end];

% Use recorded inputs as inputs to model
zero_order_hold = @(t) recorded_input(floor(t/dt)+1,:);
input_function = zero_order_hold;

% Create BabyShark model with input function
model = BabysharkModel(input_function);

% Use trim as initial conditions
y_0 = [model.u_trim 0 model.w_trim ...
    0 0 0 ...
    0 model.theta_trim ...    
    model.delta_a_trim model.delta_e_trim model.delta_r_trim];

% State: [u, v, w, p, q, r, phi, theta, delta_a, delta_e, delta_r];

[t_sim, y_sim] = ode45(@(t,y) model.f(t, y), tspan, y_0);
plot(t_sim, y_sim);