clc; close all; clear all;

%%%%%%%%
% Example file that loads recorded inputs, simulates the model and
% visualizes the aircraft response.
%%%%%%%%

dt = 0.02;
t_end = 10;
tspan = [0 t_end-dt];
t = 0:dt:t_end-dt;

% Create BabyShark model with zero_order_hold as input function
model = BabysharkModel();

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