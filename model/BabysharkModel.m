classdef BabysharkModel
    % BabysharkModel is a full dynamic model for the Babyshark VTOL UAV.
    %
    % Simulates the 12 dimensional rigid-body equations of motion that
    % govern the aircraft motion. The nonlinear aerodynamic model is
    % derived from flight-test data around trim conditions.
    % In addition, the model simulates the control surface dynamics for the
    % three control surfaces delta_a, delta_e, delta_r.
    %
    % State: [n, e, d, u, v, w, p, q, r, phi, theta, psi, delta_a, delta_e, delta_r]
    % Input: [delta_a_sp delta_e_sp delta_r_sp delta_t delta_mr_1 delta_mr_2 delta_mr3 delta_mr_4]
    %
    % delta_a, delta_e, delta_r denote the actual control surface
    % deflections in radians.
    %
    % delta_a_sp, delta_e_sp, delta_r_sp denote their respective setpoints
    % in radians.
    %
    % delta_t, delta_mr_i, i = [1,4], denotes the squared RPS (rev/second)
    % for the fixed-wing and multirotor propellers.
    %
    % Note: that for small airspeeds (V < 1), the AoA and SSA will are set
    % equal to 0 to avoid numerical problems in this flight regime.
    % 
    %
    % model =  BabysharkModel(input_function)
    %   creates a nonlinear model of the Babyshark 260 VTOL.
    %   @arg input_function:
    %       Function that will be called to get the input to the model.
    %       input_function is expected to take the current time and state
    %       (t, x) as input, and return the 8-dimensional input vector.
    
    properties

        input_function;
        
        % Airframe properties
        b = 2.5;
        c_bar = 0.242;
        S = 0.6617;
        mass = 12.14;
        J_yy = 1.0664;
        gam = [0.1419    0.8795    1.3851    0.1045    0.9003    0.1197   -0.1872    0.5990];
        
        % Multirotor propeller placements relative to cg
        r_1_x = 0.353;
        r_1_y = 0.400;
        r_2_x = -0.447;
        r_2_y = -0.400;
        r_3_x = 0.353;
        r_3_y = -0.400;
        r_4_x = -0.447;
        r_4_y = 0.400;

        % Physics constants
        rho = 1.225;
        g = 9.81;

        % Actuator properties
        delta_rate_lim = 3.4907;
        T_servo = 0.0280;
        D_fw = 0.3810;
        c_T_fw = 0.0840;
        D_mr = 0.4064;
        c_T_mr = 0.0994;
        c_Q_mr = 0.006338;
        
        delta_a_max = 25 / 180 * pi;
        delta_e_max = 25 / 180 * pi;
        delta_r_max = 22 / 180 * pi;
        
        % Trim parameters
        V_trim = 21;
        delta_a_trim = 0.0529;
        delta_e_trim = -0.0985;
        delta_r_trim = 0;
        delta_t_trim = 110^2;

        alpha_trim = 3 / 180 * pi;
        u_trim;
        w_trim;
        theta_trim;
        
        % Longitudinal coefficients
        c_D_0 = 0.082023347170533;
        c_D_alpha = 0.271784759313426;
        c_D_alpha_sq = 1.809716833956921;
        c_D_q_hat = 10.102475948666859;
        c_D_delta_e = 0.131767698434601;
        c_D_delta_e_alpha = 0.449628082114063;

        c_L_0 = 0.460589954781376;
        c_L_alpha = 5.325333674058498;
        c_L_alpha_sq = -3.969259412355321;
        c_L_delta_e = 0.521133498717410;

        c_m_0 = 0.094975972997081;
        c_m_alpha = -1.494697885250846;
        c_m_q_hat = -13.140206987350748;
        c_m_delta_e = -0.675439877822195;

        % Lateral-Directional coefficients
        c_Y_0 = 0.010753197165620;
        c_Y_beta = -0.730985927233147;
        c_Y_p_hat = 1.077758243821209;
        c_Y_delta_a = -0.341450055180717;
        c_Y_delta_r = 0.337147629851735;

        c_l_0 = 0.000410646012381;
        c_l_beta = -0.035352240429249; 
        c_l_p_hat = -0.241855568589545;
        c_l_r_hat = 0.095258856844916;
        c_l_delta_a = 0.123591366997565;

        c_n_0 = 0.001061377644348;
        c_n_beta = 0.075888061894250;
        c_n_p_hat = -0.082343715008180;
        c_n_r_hat = -0.075229235170663;
        c_n_delta_r = -0.053717237446543;

        % Rudder-Pitch coupling coefficient
        c_m_delta_r_sq = -0.736842105263158;
    end
    methods
        function obj = BabysharkModel(input_function)
            % Store input function
            obj.input_function = input_function;
            
            % Calculate trim values that are inferred from other constants
            obj.u_trim = obj.V_trim / (sqrt(1 + tan(obj.alpha_trim)^2));
            obj.w_trim = sqrt(obj.V_trim^2 - obj.u_trim^2);
            obj.theta_trim = obj.alpha_trim;
        end

        function x_dot = f(obj, t, x)
            % Unpack states
            x_cell = num2cell(x);
            [n, e, d, u, v, w, p, q, r, phi, theta, psi, delta_a, delta_e, delta_r] = x_cell{:};

            % Unpack inputs
            input = obj.input_function(t, x);
            input_cell = num2cell(input);
            [delta_a_sp, delta_e_sp, delta_r_sp, delta_t,...
                delta_mr_1, delta_mr_2, delta_mr_3, delta_mr_4] = input_cell{:};

            % Calculate control surfaces
            delta_a_dot = obj.bound(-delta_a / obj.T_servo + delta_a_sp / obj.T_servo, -obj.delta_rate_lim, obj.delta_rate_lim);
            delta_e_dot = obj.bound(-delta_e / obj.T_servo + delta_e_sp / obj.T_servo, -obj.delta_rate_lim, obj.delta_rate_lim);
            delta_r_dot = obj.bound(-delta_r / obj.T_servo + delta_r_sp / obj.T_servo, -obj.delta_rate_lim, obj.delta_rate_lim);

            % Constrain control surfaces to max angle
            if abs(delta_a) >= obj.delta_a_max
                delta_a_dot = 0;
            end
            if abs(delta_e) >= obj.delta_e_max
                delta_e_dot = 0;
            end
            if abs(delta_r) >= obj.delta_r_max
                delta_r_dot = 0;
            end
            
            % Model is around perturbation control surface deflections
            delta_e_pert = (delta_e - obj.delta_e_trim);
            delta_r_pert = (delta_r - obj.delta_r_trim);
            delta_a_pert = (delta_a - obj.delta_a_trim);

            % Aerodynamic quantities
            V = sqrt(u^2 + v^2 + w^2);
            q_bar = (1/2) * obj.rho * V^2;
            if V < 1
                alpha = 0; % neglect aoa and ssa at small airspeeds
                beta = 0;
            else
                alpha = atan(w/u);
                beta = asin(v/V);
            end

            % Nondimensionalize rates
            p_hat = obj.b * p / (2 * obj.V_trim);
            q_hat = obj.c_bar * q / (2 * obj.V_trim);
            r_hat = obj.b * r / (2 * obj.V_trim);

            % Calculate aerodynamic coefficients
            c_D = obj.c_D_0 ...
                + obj.c_D_alpha * alpha ...
                + obj.c_D_alpha_sq * alpha^2 ...
                + obj.c_D_q_hat * q_hat ...
                + obj.c_D_delta_e * delta_e_pert ...
                + obj.c_D_delta_e_alpha * alpha * delta_e_pert;
            c_L = obj.c_L_0 ...
                + obj.c_L_alpha * alpha ...
                + obj.c_L_alpha_sq * alpha^2 ...
                + obj.c_L_delta_e * delta_e_pert;
            c_m = obj.c_m_0 ...
                + obj.c_m_alpha * alpha ...
                + obj.c_m_q_hat * q_hat ...
                + obj.c_m_delta_e * delta_e_pert ...
                + obj.c_m_delta_r_sq * delta_r.^2;

            c_Y = obj.c_Y_0 ...
                + obj.c_Y_beta * beta ...
                + obj.c_Y_p_hat * p_hat ...
                + obj.c_Y_delta_a * delta_a_pert ...
                + obj.c_Y_delta_r * delta_r_pert;
            c_l = obj.c_l_0 ...
                + obj.c_l_beta * beta ...
                + obj.c_l_p_hat * p_hat ...
                + obj.c_l_r_hat * r_hat ...
                + obj.c_l_delta_a * delta_a_pert;
            c_n = obj.c_n_0 ...
                + obj.c_n_beta * beta ...
                + obj.c_n_p_hat * p_hat ...
                + obj.c_n_r_hat * r_hat ...
                + obj.c_n_delta_r * delta_r_pert;

            % Calculate forces and moments
            D = q_bar * obj.S * c_D;
            L = q_bar * obj.S * c_L;
            m = q_bar * obj.S * obj.c_bar * c_m;
            X = -cos(alpha) * D + sin(alpha) * L;
            Z = -sin(alpha) * D - cos(alpha) * L;

            Y = q_bar * obj.S * c_Y;
            l = q_bar * obj.S * obj.b * c_l;
            n = q_bar * obj.S * obj.b * c_n;
            
            % Propeller force
            T = obj.rho * obj.D_fw^4 * obj.c_T_fw * delta_t;
            
            % Multirotor forces and moments
            T_mr_1 = obj.rho * obj.D_mr^4 * obj.c_T_mr * delta_mr_1;
            T_mr_2 = obj.rho * obj.D_mr^4 * obj.c_T_mr * delta_mr_2;
            T_mr_3 = obj.rho * obj.D_mr^4 * obj.c_T_mr * delta_mr_3;
            T_mr_4 = obj.rho * obj.D_mr^4 * obj.c_T_mr * delta_mr_4;

            T_mr = T_mr_1 + T_mr_3 + T_mr_3 + T_mr_4;
            
            Q_mr_1 = obj.rho * obj.D_mr^5 * obj.c_Q_mr * delta_mr_1;
            Q_mr_2 = obj.rho * obj.D_mr^5 * obj.c_Q_mr * delta_mr_2;
            Q_mr_3 = obj.rho * obj.D_mr^5 * obj.c_Q_mr * delta_mr_3;
            Q_mr_4 = obj.rho * obj.D_mr^5 * obj.c_Q_mr * delta_mr_4;
            
            tau_x_mr = -(obj.r_1_y * T_mr_1 + obj.r_2_y * T_mr_2 ...
                + obj.r_3_y * T_mr_3 + obj.r_4_y * T_mr_4);
            tau_y_mr = obj.r_1_x * T_mr_1 + obj.r_2_x * T_mr_2 ...
                + obj.r_3_x * T_mr_3 + obj.r_4_x * T_mr_4;
            tau_z_mr = Q_mr_1 + Q_mr_2 - Q_mr_3 - Q_mr_4;
            
            tau_x = l + tau_x_mr;
            tau_y = m + tau_y_mr;
            tau_z = n + tau_z_mr;
            
            % Kinematics
            R = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(psi)*sin(theta);
                 cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                 -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
            pos_dot = R * [u; v; w];
            n_dot = pos_dot(1);
            e_dot = pos_dot(2);
            d_dot = pos_dot(3);
            
            phi_dot = p + tan(theta) * (q * sin(phi) + r * cos(phi));
            theta_dot = q * cos(phi) - r * sin(phi);
            psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta);

            % Dynamics
            f_x = X + T - obj.mass*obj.g * sin(theta);
            f_y = Y + obj.mass*obj.g * sin(phi) * cos(theta);
            f_z = Z - T_mr + obj.mass*obj.g * cos(phi) * cos(theta);

            u_dot = r*v - q*w + (1/obj.mass) * (f_x);
            v_dot = p*w - r*u + (1/obj.mass) * (f_y);
            w_dot = q*u - p*v + (1/obj.mass) * (f_z);

            p_dot = obj.gam(1)*p*q - obj.gam(2)*q*r + obj.gam(3) * tau_x + obj.gam(4) * tau_z;
            q_dot = obj.gam(5)*p*r - obj.gam(6)*(p^2 - r^2) + (1/obj.J_yy) * tau_y;
            r_dot = obj.gam(7)*p*q - obj.gam(1)*q*r + obj.gam(4) * tau_x + obj.gam(8) * tau_z;

            x_dot = [n_dot e_dot d_dot u_dot v_dot w_dot p_dot q_dot r_dot phi_dot theta_dot psi_dot delta_a_dot delta_e_dot delta_r_dot]';
        end

        % Return bounded value clipped between bl and bu
        function y = bound(~, x,bl,bu)
          y = min(max(x,bl),bu);
        end
    end
end