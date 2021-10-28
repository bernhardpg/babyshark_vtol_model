clc; close all; clear all;

% NOTE: Remember to add all the folders to path before running

%%%%%%%%
% Example file that simulates the model with a simple feedback controller
% and visualizes the simulated response.
%
% The feedback controller is a very simple PD-controller which
% stabilizes the aircraft around a constant bank angle. It only serves the
% purpose of illustrating how to implement a feedback controller with the
% model, and is not tuned or optimized in any way.
%%%%%%%,

dt = 0.02;
t_end = 50;
tspan = [0 t_end-dt];
t = 0:dt:t_end-dt;

% Create BabyShark model with zero_order_hold as input function
model = BabysharkModel(@(t, x) pd_controller(t, x));

% Use trim speed and pitch as initial conditions
initial_height = 100;
y_0 = [0 0 -initial_height ...
       model.u_trim 0 model.w_trim ...
       0 0 0 ...
       0 model.theta_trim 0 ...    
       model.delta_a_trim model.delta_e_trim model.delta_r_trim];

% Simulate model
[t_sim, y_sim] = ode45(@(t,y) model.f(t, y), tspan, y_0);

% Visualize trajectory
visualizer = AircraftVisualizer();
visualizer.plot_trajectory(t_sim, y_sim);

function input = pd_controller(t, x)
    input = zeros(1,8);
    
    % Unpack states
    x = num2cell(x);
    [n, e, d, u, v, w, p, q, r, phi, theta, psi, delta_a, delta_e, delta_r] = x{:};
    
    phi_0 = 30 / 180 * pi;
    theta_0 = 0 / 180 * pi;
    
    delta_a_sp = - 5 * (phi - phi_0) - 5 * p;
    delta_e_sp = -(- 5 * (theta - theta_0) - 5 * q);
    
    delta_t_sp = 110^2; % just some throttle that keeps the aircraft mostly at constant altitude
    
    input(1) = delta_a_sp;
    input(2) = delta_e_sp;
    input(4) = delta_t_sp;
end