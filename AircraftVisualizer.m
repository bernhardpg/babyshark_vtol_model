classdef AircraftVisualizer
    properties
        Model3D
        TextHandles = {};
        aircraft_transformation
        aircraft_dimensions
        ax_3d
        input_axs
        fig
        timestep = 0.02
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
            obj = obj.create_ax_object();
            obj.aircraft_transformation = hgtransform('Parent', obj.ax_3d); % Transformation object that is used to rotate the aircraft            
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
            axis(obj.ax_3d, 'equal');
            viewbox = [-1 1 -1 1 -1 1] * 2 * obj.aircraft_dimensions;
            axis(obj.ax_3d, viewbox);
            set(gcf,'Color',[1 1 1])
            view(obj.ax_3d, [30 10])
            camlight(obj.ax_3d, 'left');
            material(obj.ax_3d, 'dull');
        end
        
        function plot_aircraft(obj)
            patch(obj.ax_3d, 'Faces', obj.Model3D.stl_data.faces, ...
                'Vertices', obj.Model3D.stl_data.vertices, ...
                 'FaceColor', obj.Model3D.color, ...
                 'FaceAlpha', obj.Model3D.alpha, ...
                 'EdgeColor',       'none',        ...
                 'FaceLighting',    'gouraud',     ...
                 'AmbientStrength', 0.15,...
                 'Parent', obj.aircraft_transformation); hold on
             scatter3(obj.ax_3d, 0,0,0,'filled');
             obj.render_plot();
        end
        
        function [t, x] = prepare_trajectory(obj, t_trajectory, x_trajectory)
            % Make sure dt is constant for trajectory
            t_0 = t_trajectory(1);
            t_end = t_trajectory(end);
            t = t_0:obj.timestep:t_end;
            x = interp1(t_trajectory,x_trajectory,t); 
        end
        
        function obj = plot_text(obj)
            obj.TextHandles.time_text_hdl = text(obj.ax_3d, 0.55 * obj.aircraft_dimensions * 1.5, 0, 1 * obj.aircraft_dimensions * 1.5, ...
                't = 0 sec',...
                'FontSize', 20);
            obj.TextHandles.n_text_hdl = text(obj.ax_3d, 0.55 * obj.aircraft_dimensions * 1.5, 0, 0.9 * obj.aircraft_dimensions * 1.5, ...
                'north = 0 m',...
                'FontSize', 20);
            obj.TextHandles.e_text_hdl = text(obj.ax_3d, 0.55 * obj.aircraft_dimensions * 1.5, 0, 0.8 * obj.aircraft_dimensions * 1.5, ...
                'east = 0 m',...
                'FontSize', 20);
            obj.TextHandles.h_text_hdl = text(obj.ax_3d, 0.55 * obj.aircraft_dimensions * 1.5, 0, 0.7 * obj.aircraft_dimensions * 1.5, ...
                'height = 0 m',...
                'FontSize', 20);
        end
        
        function plot_trajectory(obj, t_trajectory, x_trajectory)
            [t, x] = obj.prepare_trajectory(t_trajectory, x_trajectory);

            n = x(:,1);
            e = x(:,2);
            d = x(:,3);
            phi = x(:,10);
            theta = x(:,11);
            psi = x(:,12);
            
            obj.plot_aircraft();
            obj = obj.plot_text();
           
            tic;
            for i = 1:length(t)
                % Rotate the aircraft rigid-body
                Mx = makehgtform('xrotate', phi(i));
                My = makehgtform('yrotate', -theta(i));
                Mz = makehgtform('zrotate', -psi(i));
                set(obj.aircraft_transformation, 'Matrix', Mx*My*Mz);
                
                % Update text
                set(obj.TextHandles.time_text_hdl, 'String', sprintf('t = %3.2f sec',t(i)))
                set(obj.TextHandles.n_text_hdl, 'String', sprintf('north = %3.2f m',n(i)))
                set(obj.TextHandles.e_text_hdl, 'String', sprintf('east = %3.2f m',e(i)))
                set(obj.TextHandles.h_text_hdl, 'String', sprintf('height = %3.2f m',-d(i)))

                % Control the animation speed
                if obj.timestep * i - toc > 0
                    pause(max(0, obj.timestep * i - toc))
                end
            end
        end
        
        function obj = create_ax_object(obj)
            obj.fig = figure;
            screensize = get(0,'ScreenSize');
            set(gcf,'Position', ...
                [screensize(3)/40 screensize(4)/12 screensize(3)/2*2 screensize(3)/2.2*1.0],...
                'Visible','on');
            
            % 3D animation
            obj.ax_3d = axes(obj.fig, 'position',[0.0 0.0 0.5 1]);
            axis off
            set(obj.ax_3d,'color','none');
            axis(obj.ax_3d, 'equal')
            hold on;
            
            % Aircraft inputs
            input_plot_positions = [0.5 0.7 0.2 0.15;
                                    0.5 0.5 0.2 0.15;
                                    0.5 0.3 0.2 0.15;
                                    0.5 0.1 0.2 0.15;
                                    0.75 0.7 0.2 0.15;
                                    0.75 0.5 0.2 0.15;
                                    0.75 0.3 0.2 0.15;
                                    0.75 0.1 0.2 0.15];
                                
            input_plot_names = ["\delta_a" "\delta_e", "\delta_r" "\delta_t" ...
                "\delta_{mr1}" "\delta_{mr2}" "\delta_{mr3}" "\delta_{mr4}"];    
                                    
            for i = 1:8
                obj.input_axs(i) = axes(obj.fig, 'position', input_plot_positions(i,:));
                plot(obj.input_axs(i), 1:15);
                title(obj.input_axs(i), input_plot_names(i));
            end
        end
    end
end

