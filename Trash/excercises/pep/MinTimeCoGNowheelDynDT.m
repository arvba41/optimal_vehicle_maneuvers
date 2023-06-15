classdef MinTimeCoGNowheelDynDT < handle
    % This is a single track model
    properties (SetAccess = public)
        lf;
        lr;
        l;
        w_veh;
        m;
        Izz;
        Rw;
        Iw;
        g;
        mu_xf;
        mu_xr;
        B_xf;
        B_xr;
        C_xf;
        C_xr;
        E_xf;
        E_xr;
        mu_yf;
        mu_yr;
        B_yf;
        B_yr;
        C_yf;
        C_yr;
        E_yf;
        E_yr;
        C_xaf;
        C_xar;
        B_x1f;
        B_x1r;
        B_x2f;
        B_x2r;
        C_ykf;
        C_ykr;
        B_y1f;
        B_y1r;
        B_y2f;
        B_y2r;
        N;
        Xa;
        R_xi;
        R_xo;
        Ya;
        R_yi;
        R_yo;
        p;
        Xp0;
        Yp0;
        phi0;
        vx0;
        vy0;
        r0;
        wf0;
        wr0;
        del0;
        del_min;
        del_dot_min;
        X_tf;
        Y_tf;
        A;
        b;

        optimizer;
        
    end
    
    methods (Access = public)
        function obj = MinTimeCoGNowheelDynDT(parameters)
            obj.lf = parameters.lf;
            obj.lr = parameters.lr;
            obj.l = parameters.l;
            obj.w_veh = parameters.w_veh;
            obj.m = parameters.m;
            obj.Izz = parameters.Izz;
            obj.Rw = parameters.Rw;
            obj.Iw = parameters.Iw;
            obj.g = parameters.g;
            obj.mu_xf = parameters.mu_xf;
            obj.mu_xr = parameters.mu_xr;
            obj.B_xf = parameters.B_xf;
            obj.B_xr = parameters.B_xr;
            obj.C_xf = parameters.C_xf;
            obj.C_xr = parameters.C_xr;
            obj.E_xf = parameters.E_xf;
            obj.E_xr = parameters.E_xr;
            obj.mu_yf = parameters.mu_yf;
            obj.mu_yr = parameters.mu_yr;
            obj.B_yf = parameters.B_yf;
            obj.B_yr = parameters.B_yr;
            obj.C_yf = parameters.C_yf;
            obj.C_yr = parameters.C_yr;
            obj.E_yf = parameters.E_yf;
            obj.E_yr = parameters.E_yr;
            obj.C_xaf = parameters.C_xaf;
            obj.C_xar = parameters.C_xar;
            obj.B_x1f = parameters.B_x1f;
            obj.B_x1r = parameters.B_x1r;
            obj.B_x2f = parameters.B_x2f;
            obj.B_x2r = parameters.B_x2r;
            obj.C_ykf = parameters.C_ykf;
            obj.C_ykr = parameters.C_ykr;
            obj.B_y1f = parameters.B_y1f;
            obj.B_y1r = parameters.B_y1r;
            obj.B_y2f = parameters.B_y2f;
            obj.B_y2r = parameters.B_y2r;
            obj.N = parameters.N;
            obj.Xa = parameters.Xa;
            obj.R_xi = parameters.R_xi;
            obj.R_xo = parameters.R_xo;
            obj.Ya = parameters.Ya;
            obj.R_yi = parameters.R_yi;
            obj.R_yo = parameters.R_yo;
            obj.p = parameters.p;
            obj.Xp0 = parameters.Xp0;
            obj.Yp0 = parameters.Yp0;
            obj.phi0 = parameters.phi0;
            obj.vx0 = parameters.vx0;
            obj.vy0 = parameters.vy0;
            obj.r0 = parameters.r0;
            obj.wf0 = parameters.wf0;
            obj.wr0 = parameters.wr0;
            obj.del0 = parameters.del0;
            obj.del_min = parameters.del_min;
            obj.del_dot_min = parameters.del_dot_min;
            obj.X_tf = parameters.X_tf;
            obj.Y_tf = parameters.Y_tf;
            obj.A = parameters.A;
            obj.b = parameters.b;

            obj.optimizer = obj.OPTFormulation( );
        end

        function [U_opt, X_opt, tf_opt] = solve(obj)
            X0 = [obj.Xp0; obj.Yp0; obj.phi0; obj.vx0; obj.vy0; obj.r0];
            [U_opt, X_opt, tf_opt] = obj.optimizer(X0);
            U_opt  = U_opt.full();
            X_opt  = X_opt.full();
            tf_opt = tf_opt.full();
        end
        
        function x_dot = vehicle_model(obj, x, u)

            phi = x(3);
            vx  = x(4);
            vy  = x(5);
            r   = x(6);
            del = x(7);
            F_x1  = u(1);
            F_x2 = u(1);
            [F_x3, F_x4]  = deal(0);

            del_1 = del;
            del_2 = del;
            [del_3, del_4] = deal(0);

            [l_x1, l_x2] = deal(obj.lf);
            [l_x3, l_x4] = deal(-obj.lr);
            [l_y1, l_y3] = deal(obj.w_veh/2);
            [l_y2, l_y4] = deal(-obj.w_veh/2);
            [mu_y1, mu_y2] = deal(obj.mu_yf);
            [mu_y3, mu_y4] = deal(obj.mu_yr);
            [mu_x1, mu_x2] = deal(obj.mu_xf);
            [C_y1, C_y2] = deal(obj.C_yf);
            [C_y3, C_y4] = deal(obj.C_yr);
            [B_y1, B_y2] = deal(obj.B_yf);
            [B_y3, B_y4] = deal(obj.B_yr);
            [E_y1, E_y2] = deal(obj.E_yf);
            [E_y3, E_y4] = deal(obj.E_yr);

            a1 = del - atan((vy + obj.lf*r)/vx);
            a2 = del - atan((vy + obj.lf*r)/vx);
            a3 = -atan((vy - obj.lr*r)/vx);
            a4 = -atan((vy - obj.lr*r)/vx);

            
            [F_z1, F_z2] = deal(obj.m*obj.g*(obj.l - obj.lf)/obj.l/2);
            [F_z3, F_z4] = deal(obj.m*obj.g*(obj.l - obj.lr)/obj.l/2);
            
            F_y01 = mu_y1*F_z1*sin(C_y1*atan(B_y1*a1 - E_y1*(B_y1*a1 - atan(B_y1*a1))));
            F_y02 = mu_y2*F_z2*sin(C_y2*atan(B_y2*a2 - E_y2*(B_y2*a2 - atan(B_y2*a2))));
            F_y03 = mu_y3*F_z3*sin(C_y3*atan(B_y3*a3 - E_y3*(B_y3*a3 - atan(B_y3*a3))));
            F_y04 = mu_y4*F_z4*sin(C_y4*atan(B_y4*a4 - E_y4*(B_y4*a4 - atan(B_y4*a4))));
            
            F_x10_max = mu_x1*F_z1;
            F_x20_max = mu_x2*F_z2;
            %F_xr0_max = obj.mu_xr*F_zr;

            F_y1 = F_y01*sqrt(1 - (F_x1/F_x10_max)^2);
            F_y2 = F_y02*sqrt(1 - (F_x2/F_x20_max)^2);
            F_y3 = F_y03; %*sqrt(1 - (F_xr/F_xr0_max)^2);
            F_y4 = F_y04;

            Temp = [1 0; 0 1; -l_y1 l_x1]*[cos(del_1) -sin(del_1); sin(del_1) cos(del_1)]*[F_x1; F_y1] + ...
                   [1 0; 0 1; -l_y2 l_x2]*[cos(del_2) -sin(del_2); sin(del_2) cos(del_2)]*[F_x2; F_y2] + ...
                   [1 0; 0 1; -l_y3 l_x3]*[cos(del_3) -sin(del_3); sin(del_3) cos(del_3)]*[F_x3; F_y3] + ...
                   [1 0; 0 1; -l_y4 l_x4]*[cos(del_4) -sin(del_4); sin(del_4) cos(del_4)]*[F_x4; F_y4];
            Fx = Temp(1);
            Fy = Temp(2);
            Mz = Temp(3);


            
            Xp_dot = vx*cos(phi) - vy*sin(phi);
            Yp_dot = vx*sin(phi) + vy*cos(phi);
            phi_dot = r;
            vx_dot  = Fx/obj.m + vy*r;
            vy_dot  = Fy/obj.m - vx*r;
            r_dot   = Mz/obj.Izz;
            del_dot = u(3);
            
            x_dot = [Xp_dot; Yp_dot; phi_dot; vx_dot; vy_dot; r_dot; del_dot];
        end


        function optimizer = OPTFormulation(obj)
            [mu_x1, mu_x2] = deal(obj.mu_xf);
            F_z1 = obj.m*obj.g*(obj.l - obj.lf)/obj.l/2;
            F_z2 = obj.m*obj.g*(obj.l - obj.lf)/obj.l/2;
            %F_zr = obj.m*obj.g*(obj.l - obj.lr)/obj.l;
            F_x1_min = -mu_x1*F_z1;
            F_x1_max =  mu_x1*F_z1;
            F_x2_min = -mu_x2*F_z2;
            F_x2_max =  mu_x2*F_z2;
            %F_xr_min = -obj.mu_xr*F_zr;
            %F_xr_max =  obj.mu_xr*F_zr;

            opti = casadi.Opti(); 
            U  = opti.variable(3, obj.N); 
            X  = opti.variable(7, obj.N + 1);
            Xp = X(1, :);
            Yp = X(2, :);
            vx = X(4, :);
            del = X(7, :);
            tf = opti.variable( ); 

            X0 = opti.parameter(6, 1); 
            
            opti.minimize(tf); 
            
            dt = tf/obj.N; 
            opti.subject_to(X(1:end - 1, 1) == X0);
            for k = 1:obj.N 
               k1 = obj.vehicle_model(X(:, k),          U(:, k));
               k2 = obj.vehicle_model(X(:,k) + dt/2*k1, U(:, k));
               k3 = obj.vehicle_model(X(:,k) + dt/2*k2, U(:, k));
               k4 = obj.vehicle_model(X(:,k) + dt*k3,   U(:, k));
               x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
               opti.subject_to(X(:,k+1)== x_next); 
            end

            opti.subject_to(F_x1_min <= U(1, :) <= F_x1_max);
            opti.subject_to(F_x2_min <= U(2, :) <= F_x2_max);
            opti.subject_to(obj.del_min <= del(2:end) <= -obj.del_min); 
            opti.subject_to(obj.del_dot_min <= U(3, :) <= -obj.del_dot_min); 
            opti.subject_to(-((Xp(2:end) - obj.Xa)/obj.R_xi).^obj.p - ((Yp(2:end) - obj.Ya)/obj.R_yi).^obj.p <= -1); 
            %for k = 1:1:obj.N
            %     opti.subject_to(obj.A*X(1:2, k + 1) <= obj.b);
            %end
            opti.subject_to(((Xp(2:end) - obj.Xa)/obj.R_xo).^obj.p + ((Yp(2:end) - obj.Ya)/obj.R_yo).^obj.p <= 1); 
            opti.subject_to(Xp(end) == obj.X_tf);
            opti.subject_to(Yp(end) == obj.Y_tf);
            opti.subject_to(tf >=0.1); 
            %opti.subject_to(0 <= rho_x <= 1);
            %opti.subject_to(0 <= rho_y <= 1);


            opti.set_initial(tf, 3.23);
            opti.set_initial(vx, obj.vx0);
            opti.set_initial(Yp, obj.Yp0);

            options = struct;
            %options.ipopt.linear_solver = 'ma27';
            options.ipopt.max_iter = 7000;
            opti.solver('ipopt', options); 
            optimizer = opti.to_function('f', {X0}, {U, X, tf});
        end
    end
end