clc; close all; clear all;

%%%%%%%%
% Example file that loads recorded inputs, simulates the model and
% visualizes the aircraft response.
%%%%%%%%

% Read example maneuver
recorded_input = readmatrix("example_inputs/roll_maneuver_right_input.csv");
dt = readmatrix("example_inputs/dt.csv");
t_end = length(recorded_input) * dt - dt;
tspan = [0 t_end];

% Use zero-order-hold on the recorded inputs,
% and use these as inputs to model
zero_order_hold = @(t) recorded_input(floor(t/dt)+1,:);

% dt = 0.02;
% t_end = 10;
% tspan = [0 t_end-dt];
% t = 0:dt:t_end-dt;
% recorded_input = zeros(t_end / dt, 8);

% Create BabyShark model with zero_order_hold as input function
model = BabysharkModel(zero_order_hold);

% Use trim speed and pitch as initial conditions
y_0 = [0 0 -100 ...
       model.u_trim 0 model.w_trim ...
       0 0 0 ...
       0 model.theta_trim 0 ...    
       model.delta_a_trim model.delta_e_trim model.delta_r_trim];

% Simulate model
[t_sim, y_sim] = ode45(@(t,y) model.f(t, y), tspan, y_0);

% Visualize trajectory
visualizer = AircraftVisualizer();
visualizer.plot_trajectory(t_sim, y_sim);