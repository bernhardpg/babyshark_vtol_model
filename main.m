clc; close all; clear all;

input_function = @(t) [0; 0; 0; 0];
model = BabysharkModel(input_function);

tspan = [0 10];
y_0 = [0 0 0 0 0 0 0 0 0 0 0];

[t_sim, y_sim] = ode45(@(t,y) model.f(t, y), tspan, y_0);