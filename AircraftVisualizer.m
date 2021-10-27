classdef AircraftVisualizer
    properties
        Model3D
        aircraft_transformation
        aircraft_dimensions
        ax
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
            
            obj.aircraft_dimensions = max(max(sqrt(sum(obj.Model3D.stl_data.vertices.^2, 2))));
            obj.ax = obj.create_ax_object();
            obj.aircraft_transformation = hgtransform('Parent', obj.ax); % Transformation object that is used to rotate the aircraft            
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
        
        function render_plot(obj)
            axis('equal');
            axis([-1 1 -1 1 -1 1] * 1.0 * obj.aircraft_dimensions)
            set(gcf,'Color',[1 1 1])
            axis off
            view([30 10])
            camlight('left');
            material('dull');
        end
        
        function plot_aircraft(obj)
            patch('Faces', obj.Model3D.stl_data.faces, ...
                'Vertices', obj.Model3D.stl_data.vertices, ...
                 'FaceColor', obj.Model3D.color, ...
                 'FaceAlpha', obj.Model3D.alpha, ...
                 'EdgeColor',       'none',        ...
                 'FaceLighting',    'gouraud',     ...
                 'AmbientStrength', 0.15,...
                 'Parent', obj.aircraft_transformation); hold on
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
            
            obj.plot_aircraft();
            time_text_hdl = text(0, 0, 0.55 * obj.aircraft_dimensions * 1.5, ...
                't = 0 sec',...
                'FontSize', 20);

            tic;
            for i = 1:length(t)
                % Rotate the aircraft rigid-body
                Mx = makehgtform('xrotate', phi(i));
                My = makehgtform('yrotate', -theta(i));
                Mz = makehgtform('zrotate', psi(i));
                set(obj.aircraft_transformation, 'Matrix', Mx*My*Mz);
                
                % Update text
                set(time_text_hdl, 'String', sprintf('t = %3.2f sec',t(i)))

                % Control the animation speed
                if obj.timestep * i - toc > 0
                    pause(max(0, obj.timestep * i - toc))
                end
            end
        end
        
        function ax = create_ax_object(~)
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
        end
    end
end

