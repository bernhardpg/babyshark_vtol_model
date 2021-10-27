clc; clear all; close all;

%%%% INITIALIZE AIRCRAFT VERTICES
% Import an STL mesh, returning a PATCH-compatible face-vertex structure
model = stlread('3d_files/babyshark.stl');

V = model.vertices;
F = model.faces;

% Rotate the aircraft to initial position, with positive x-axis out of nose
initial_phi = pi/2;
initial_theta = 0;
initial_psi = pi/2;
V = rotate_vertices(V, initial_phi, initial_theta, initial_psi);

% Scale the aircraft to the correct size
wingspan = 2.5;
V = scale_aircraft(wingspan, V);

% Move origin to front of aircraft nose
temp_max = max(V);
temp_min = min(V);
ranges = abs(temp_max - temp_min);
aircraft_length = ranges(1);
V = V - [aircraft_length wingspan/2 0];

% Move origin to cg
cg_position_from_front = -0.494;
cg_position_from_bottom = 0.25;
cg_position = [cg_position_from_front 0 cg_position_from_bottom];
V = V - cg_position;

plot_aircraft(F,V); hold on
scatter3(0, 0, 0)

%%%%%%%%%%%%%%%%%%%
% Render settings %
%%%%%%%%%%%%%%%%%%%

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
%view(view_angle);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on
title("Hello")


%%


%%%%%%%%%%%%%%%%%
% PLOT MANEUVER %
%%%%%%%%%%%%%%%%%

num_maneuvers = length(maneuvers_to_use);
for i = 1:num_maneuvers
    maneuver = maneuvers_to_use(i);
    plot_maneuver(plot_resolution_s, maneuver, V_0, F, view_angle)
end



function plot_maneuver(plot_resolution_s, maneuver, V_0, F, view_angle)
    fig = figure;
    fig.Position = [100 100 1500 500];
    plot_settings;

    t = maneuver.Time - maneuver.Time(1);
    dt = t(2) - t(1);
    index_interval = round(plot_resolution_s / dt);
    indices_to_plot = [1:index_interval:length(t)];

    phi = maneuver.EulPhi;
    theta = maneuver.EulTheta;
    psi_0 = maneuver.EulPsi(1);
    psi = maneuver.EulPsi - psi_0;

    N = maneuver.PosN - maneuver.PosN(1);
    E = maneuver.PosE - maneuver.PosE(1);
    D = -(maneuver.PosD);
    p = [N E D];
    p = rotate_vertices(p, 0, 0, -psi_0);

    for i = indices_to_plot
       V_curr = rotate_vertices(V_0, phi(i), -theta(i), psi(i));
       V_curr = V_curr + p(i,:);
       plot_aircraft(F, V_curr);
       
       text_pos = p(i,:) + [0 0 2.5];
       text(text_pos(1), text_pos(2), text_pos(3), "t = " + t(i) + "s", 'interpreter', 'Latex','FontSize',font_size);
    end

    %%%%%%%%%%%%%%%%%%%
    % Render settings %
    %%%%%%%%%%%%%%%%%%%

    % Add a camera light, and tone down the specular highlighting
    camlight('headlight');
    material('dull');

    % Fix the axes scaling, and set a nice view angle
    axis('image');
    view(view_angle);
    xlabel('x [m]', 'interpreter', 'Latex','FontSize',font_size)
    ylabel('y [m]', 'interpreter', 'Latex','FontSize',font_size)
    zlabel('z [m]', 'interpreter', 'Latex','FontSize',font_size)
    grid on
    title("Hello", 'FontSize', font_size_large, 'interpreter', 'latex')
end

function plot_aircraft(F, V)
    patch('Faces', F, 'Vertices', V, ...
         'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15); hold on
end

function V_rotated = rotate_vertices(V, phi, theta, psi)
    Rx = [1 0 0;
          0 cos(phi) -sin(phi);
          0 sin(phi) cos(phi)];

    Ry = [cos(theta) 0 sin(theta);
          0 1 0;
          -sin(theta) 0 cos(theta)];

    Rz = [cos(psi), -sin(psi), 0 ;
          sin(psi), cos(psi), 0 ;
                 0,         0, 1 ];

    V_rotated = V * Rx';
    V_rotated = V_rotated * Ry';
    V_rotated = V_rotated * Rz';
end

function V_centered = center_vertices(V)
    temp_max = max(V);
    temp_min = min(V);
    ranges = abs(temp_max - temp_min);
    translation = ranges / 2;
    V_centered = V - translation;
end

function V_scaled = scale_aircraft(target_wingspan_m, V)
    temp_max = max(V);
    temp_min = min(V);
    ranges = abs(temp_max - temp_min);
    y_range = ranges(2);
    scaling_factor = y_range / target_wingspan_m;
    V_scaled = V / scaling_factor;
end