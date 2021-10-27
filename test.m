close all; clc; close all;

ax = axes('XLim', [-1.5 1.5], 'YLim', [-1.5 1.5], 'ZLim', [-1.5 1.5]);
axis(ax, 'equal')
view(3)
grid on; box on;

t = hgtransform('Parent', ax);
[x,y,z] = cylinder([.2 0]);
h(1) = surface( x,  y,  z, 'FaceColor', 'white');
h(2) = surface( x,  y, -z, 'FaceColor', 'white');
h(3) = surface( z,  x,  y, 'FaceColor', 'green');
h(4) = surface(-z,  x,  y, 'FaceColor', 'red');
h(5) = surface( y,  z,  x, 'FaceColor', 'white');
h(6) = surface( y, -z,  x, 'FaceColor', 'white');
set(h, 'Parent', t)
tic;
for r = 1:.1:2*pi
    % Z-axis rotation matrix
    Rz = makehgtform('zrotate', r);
    % Scaling matrix
    Sxy = makehgtform('scale', r/4);
    % Concatenate the transforms and
    % set the transform Matrix property
    set(t, 'Matrix',Rz*Sxy)
    drawnow limitrate;
    % Control the animation speed
    if dt * 10 * r - toc > 0
        pause(max(0, 10 * dt * r - toc))
    end
end