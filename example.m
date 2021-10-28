clc; close all; clear all;

%%%%%%%%
% Example file that loads recorded inputs and simulates the model
%%%%%%%%

% Read example maneuver
recorded_input = readmatrix("example_inputs/roll_maneuver_right_input.csv");
dt = readmatrix("example_inputs/dt.csv");
t_end = length(recorded_input) * dt - dt;
tspan = [0 t_end];

% dt = 0.02;
% t_end = 10;
% tspan = [0 t_end-dt];
% t = 0:dt:t_end-dt;
% recorded_input = zeros(t_end / dt, 8);

% Use recorded inputs as inputs to model
zero_order_hold = @(t) recorded_input(floor(t/dt)+1,:);
input_function = zero_order_hold;

% Create BabyShark model with input function
model = BabysharkModel(input_function);

% Use trim as initial conditions
% State: [n, e, d, u, v, w, p, q, r, phi, psi, theta, delta_a, delta_e, delta_r];
y_0 = [0 0 -100 ...
    0 0 0 ...
    0 0 0 ...
    0 0 0 ...    
    model.delta_a_trim model.delta_e_trim model.delta_r_trim];

% Simulate model
[t_sim, y_sim] = ode45(@(t,y) model.f(t, y), tspan, y_0);

% Visualize trajectory
visualizer = AircraftVisualizer();
visualizer.plot_trajectory(t_sim, y_sim);


