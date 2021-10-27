classdef BabysharkModel
    % State: [u, v, w, p, q, r, phi, theta, delta_a, delta_e, delta_r]
    % Input: TODO
    
    properties
        % Airframe properties
        b = 2.5;
        c_bar = 0.242;
        S = 0.6617;
        mass = 12.14;
        J_yy = 1.0664;
        gam = [0.1419    0.8795    1.3851    0.1045    0.9003    0.1197   -0.1872    0.5990];

        % Physics constants
        rho = 1.225;
        g = 9.81;

        % Actuator properties
        delta_rate_lim = 3.4907;
        T_servo = 0.0280;
        D_FW = 0.3810;
        c_T_FW = 0.0840;
        
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
        
        input_function; % Function that will be called to get input to the model
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
            x = num2cell(x);
            [u, v, w, p, q, r, phi, theta, delta_a, delta_e, delta_r] = x{:};

            % Unpack inputs
            input = obj.input_function(t);
            input = num2cell(input);
            [delta_a_sp, delta_e_sp, delta_r_sp, delta_t,...
                delta_mr_1, delta_mr_2, delta_mr_3, delta_mr_4] = input{:};

            % Calculate control surfaces
            delta_a_dot = obj.bound(-delta_a / obj.T_servo + delta_a_sp / obj.T_servo, -obj.delta_rate_lim, obj.delta_rate_lim);
            delta_e_dot = obj.bound(-delta_e / obj.T_servo + delta_e_sp / obj.T_servo, -obj.delta_rate_lim, obj.delta_rate_lim);
            delta_r_dot = obj.bound(-delta_r / obj.T_servo + delta_r_sp / obj.T_servo, -obj.delta_rate_lim, obj.delta_rate_lim);

            % Model is around perturbation control surface deflections
            delta_e_pert = (delta_e - obj.delta_e_trim);
            delta_r_pert = (delta_r - obj.delta_r_trim);
            delta_a_pert = (delta_a - obj.delta_a_trim);

            % Aerodynamic quantities
            V = sqrt(u^2 + v^2 + w^2);
            q_bar = (1/2) * obj.rho * V^2;
            alpha = atan(w/u);
            beta = asin(v/V);

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
            T = obj.rho * obj.D_FW^4 * obj.c_T_FW * delta_t;

            % Dynamics
            phi_dot = p + tan(theta) * (q * sin(phi) + r * cos(phi));
            theta_dot = q * cos(phi) - r * sin(phi);

            f_x = X + T - obj.mass*obj.g * sin(theta);
            f_y = Y + obj.mass*obj.g * sin(phi) * cos(theta);
            f_z = Z + obj.mass*obj.g * cos(phi) * cos(theta);

            u_dot = r*v - q*w + (1/obj.mass) * (f_x);
            v_dot = p*w - r*u + (1/obj.mass) * (f_y);
            w_dot = q*u - p*v + (1/obj.mass) * (f_z);

            p_dot = obj.gam(1)*p*q - obj.gam(2)*q*r + obj.gam(3)*l + obj.gam(4)*n;
            q_dot = obj.gam(5)*p*r - obj.gam(6)*(p^2 - r^2) + (1/obj.J_yy) * m;
            r_dot = obj.gam(7)*p*q - obj.gam(1)*q*r + obj.gam(4)*l + obj.gam(8)*n;

            x_dot = [u_dot v_dot w_dot p_dot q_dot r_dot phi_dot theta_dot delta_a_dot delta_e_dot delta_r_dot]';
        end

        % return bounded value clipped between bl and bu
        function y = bound(obj, x,bl,bu)
          y = min(max(x,bl),bu);
        end
    end
end