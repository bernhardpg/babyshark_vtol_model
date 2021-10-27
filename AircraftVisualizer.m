classdef AircraftVisualizer
    properties
        Model3D
        timestep = 0.1
    end
    
    methods
        function obj = AircraftVisualizer()
            obj.Model3D.cg_position_from_front = -0.494;
            obj.Model3D.cg_position_from_bottom = 0.25;
            obj.Model3D.wingspan = 2.5;
            obj.Model3D.alpha = 0.8;
            obj.Model3D.color = [0.8 0.8 1.0];
            
            model = stlread('3d_files/babyshark.stl');
            obj.Model3D.stl_data.vertices = model.vertices;
            obj.Model3D.stl_data.faces = model.faces;
            obj = obj.initialize_aircraft_model();
        end
        function obj = initialize_aircraft_model(obj)
            % Import an STL mesh, returning a PATCH-compatible face-vertex structure
            V0 = obj.Model3D.stl_data.vertices;

            % Rotate the aircraft to initial position, with positive x-axis out of nose
            initial_phi = pi/2;
            initial_theta = 0;
            initial_psi = pi/2;
            V0 = obj.rotate_vertices(V0, initial_phi, initial_theta, initial_psi);

            % Scale the aircraft to the correct size
            V0 = obj.scale_aircraft(obj.Model3D.wingspan, V0);

            % Move origin to front of aircraft nose
            temp_max = max(V0);
            temp_min = min(V0);
            ranges = abs(temp_max - temp_min);
            aircraft_length = ranges(1);
            V0 = V0 - [aircraft_length obj.Model3D.wingspan/2 0];

            % Move origin to cg
            cg_position = [...
                obj.Model3D.cg_position_from_front ...
                0 ...
                obj.Model3D.cg_position_from_bottom...
                ];
            V0 = V0 - cg_position; 
            
            obj.Model3D.stl_data.vertices = V0;
        end
        
        function V_scaled = scale_aircraft(~, wingspan, V)
            temp_max = max(V);
            temp_min = min(V);
            ranges = abs(temp_max - temp_min);
            y_range = ranges(2);
            scaling_factor = y_range / wingspan;
            V_scaled = V / scaling_factor;
        end
        
        function V_rotated = rotate_vertices(~, V, phi, theta, psi)
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
        
        function render_plot(~)
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
        end
        
        function plot_aircraft(obj)
            patch('Faces', obj.Model3D.stl_data.faces, ...
                'Vertices', obj.Model3D.stl_data.vertices, ...
                 'FaceColor', obj.Model3D.color, ...
                 'FaceAlpha', obj.Model3D.alpha, ...
                 'EdgeColor',       'none',        ...
                 'FaceLighting',    'gouraud',     ...
                 'AmbientStrength', 0.15); hold on
             scatter3(0,0,0,'filled');
             
             obj.render_plot();
        end
        
        function [t, x] = prepare_trajectory(obj, t_trajectory, x_trajectory)
            % Make sure dt is constant for trajectory
            t_0 = t_trajectory(1);
            t_end = t_trajectory(end);
            t = t_0:obj.timestep:t_end;
            x = interp1(t_trajectory,x_trajectory,t); 
        end
        
        function plot_trajectory(obj, t_trajectory, x_trajectory)
            [t, x] = obj.prepare_trajectory(t_trajectory, x_trajectory);

            phi = x(:,7);
            theta = x(:,8);
            psi = zeros(size(phi));

            % Define the figure properties
            ax = axes('position',[0.0 0.0 1 1]);
            axis off
            screensize = get(0,'ScreenSize');
            set(gcf,'Position', ...
            [screensize(3)/40 screensize(4)/12 screensize(3)/2*1.0 screensize(3)/2.2*1.0],...
            'Visible','on');
            set(ax,'color','none');
            axis('equal')
            hold on;
            cameratoolbar('Show')

            aircraft_transf = hgtransform('Parent', ax);
            patch('Faces', obj.Model3D.stl_data.faces, ...
            'Vertices', obj.Model3D.stl_data.vertices, ...
             'FaceColor', obj.Model3D.color, ...
             'FaceAlpha', obj.Model3D.alpha, ...
             'EdgeColor',       'none',        ...
             'FaceLighting',    'gouraud',     ...
             'AmbientStrength', 0.15,...
             'Parent', aircraft_transf); % Assign the aircraft to the transformation
            scatter3(0,0,0,'filled');

            % Fixing the axes scaling and setting a nice view angle
            axis('equal');
            aircraft_dimensions = max(max(sqrt(sum(obj.Model3D.stl_data.vertices.^2, 2))));
            axis([-1 1 -1 1 -1 1] * 1.0 * aircraft_dimensions)
            set(gcf,'Color',[1 1 1])
            axis off
            view([30 10])
            % Add a camera light, and tone down the specular highlighting
            camlight('left');
            material('dull');

            tic;
            for i = 1:length(t)
                % Rotate the aircraft rigid-body
                Mx = makehgtform('xrotate', phi(i));
                My = makehgtform('yrotate', -theta(i));
                Mz = makehgtform('zrotate', psi(i));
                set(aircraft_transf, 'Matrix', Mx*My*Mz);

                % Control the animation speed
                if obj.timestep * i - toc > 0
                    pause(max(0, obj.timestep * i - toc))
                end
            end

        end
        
        function lol()
            t = hgtransform('Parent', ax);
            
            hsp = subplot(1, 1, 1);
            grid(hsp, 'on');
            box(hsp, 'on');
            tic;
            for i = 1:length(t)
                hold(hsp, 'off');
                %plot(t(1:i), phi(1:i), '-b');
                obj.plot_aircraft(obj.model_vertices_0)
                hold(hsp, 'on');
                text(0.5,0.5,"t = " + t(i), 'FontSize', 20);
                xlim(hsp, [0, t_end]);
                ylim(hsp, [0, +1.2]);
                pause(obj.timestep)
            end
            
        end
    end
end

