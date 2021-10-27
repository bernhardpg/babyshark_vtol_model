clearvars;
close all;

% Define the list of parts which will be part of the rigid aircraft body
rigid_body_list   = {'babyshark.stl'};
% Define the color of each part
rigid_body_colors = {0.8 * [1, 1, 1]};
% Define the transparency of each part
alphas            = [              1];
% Define the model offset vector to center the A/C body at the Center of Gravity
% (CG)
offset_3d_model   = [8678.85, -15.48, 606.68];
% Rigid body parts
for i = 1:length(rigid_body_list)
    Model3D.Aircraft(i).model = rigid_body_list{i};
    Model3D.Aircraft(i).color = rigid_body_colors{i};
    Model3D.Aircraft(i).alpha = alphas(i);
    % Read the *.stl file
   [Model3D.Aircraft(i).stl_data.vertices, Model3D.Aircraft(i).stl_data.faces, ~, Model3D.Aircraft(i).label] = stlRead(rigid_body_list{i});
    Model3D.Aircraft(i).stl_data.vertices  = Model3D.Aircraft(i).stl_data.vertices - offset_3d_model;
end