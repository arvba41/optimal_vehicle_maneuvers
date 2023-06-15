classdef MinTimeCoGNoFx < handle
    
    properties (SetAccess = public)
        lf;
        lr;
        l;
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
        C_af;
        C_ar;
        N;
        Xa;
        Rx;
        Ya;
        Ry;
        p;

        optimizer;
        
    end
    
    methods (Access = public)
        function obj = MinTimeCoGNoFx(parameters)
            obj.lf = parameters.lf;
            obj.lr = parameters.lr;
            obj.l = parameters.l;
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
            obj.C_af = parameters.C_af;
            obj.C_ar = parameters.C_ar;
            obj.N = parameters.N;
            obj.Xa = parameters.Xa;
            obj.Rx = parameters.Rx;
            obj.Ya = parameters.Ya;
            obj.Ry = parameters.Ry;
            obj.p = parameters.p;

            obj.optimizer = obj.OPTFormulation( );
        end

        function [U_opt, X_opt, tf_opt] = solve(obj, X0, del_min, X_tf, Y_tf)
            [U_opt, X_opt, tf_opt] = obj.optimizer(X0, del_min, X_tf, Y_tf);
            U_opt  = U_opt.full();
            X_opt  = X_opt.full();
            tf_opt = tf_opt.full();

        end
        
        function x_dot = vehicle_model(obj, x, u)
            
            vx = 30;
            vy  = x(4);
            phi = x(5);
            r   = x(6);
            
            Xp_dot = vx*cos(phi) - vy*sin(phi);
            Yp_dot = vx*sin(phi) + vy*cos(phi);
            y_dot  = vy;
            vy_dot = ((obj.C_ar + obj.C_af)/obj.m/vx)*vy + ((obj.C_af*obj.lf - obj.C_ar*obj.lr)/obj.m/vx - vx)*r - obj.C_af/obj.m*u;
            phi_dot = r;
            r_dot = (obj.C_af*obj.lf - obj.C_ar*obj.lr)/obj.Izz/vx*vy + (obj.C_ar*obj.lr^2 + obj.C_af*obj.lf^2)/obj.Izz/vx*r - obj.C_af*obj.lf/obj.Izz*u;

            x_dot = [Xp_dot; Yp_dot; y_dot; vy_dot; phi_dot; r_dot];
        end


        function optimizer = OPTFormulation(obj)
            opti = casadi.Opti(); 
            U  = opti.variable(1, obj.N); 
            X  = opti.variable(6, obj.N + 1);
            tf = opti.variable( ); 

            X0      = opti.parameter(6, 1);
            del_min = opti.parameter( );
            X_tf = opti.parameter( );
            Y_tf = opti.parameter( );
            
            dt = tf/obj.N; 
            opti.subject_to(X(:, 1) == X0);
            for k = 1:obj.N 
               k1 = obj.vehicle_model(X(:, k),          U(:, k));
               k2 = obj.vehicle_model(X(:,k) + dt/2*k1, U(:, k));
               k3 = obj.vehicle_model(X(:,k) + dt/2*k2, U(:, k));
               k4 = obj.vehicle_model(X(:,k) + dt*k3,   U(:, k));
               x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
               opti.subject_to(X(:,k+1)== x_next); 
            end
            
            opti.subject_to(del_min <= U); 
            opti.subject_to(U <= -del_min);  
            opti.subject_to(X(2, end) == Y_tf);
            opti.subject_to(X(1, end) == X_tf);
            opti.subject_to(-((X(1, 2:end) - obj.Xa)/obj.Rx).^obj.p - ((X(2, 2:end) - obj.Ya)/obj.Ry).^obj.p <= -1);  
            opti.subject_to(tf >=0); 
            
            opti.set_initial(tf, 10);

            opti.minimize(tf + 1500000*U*U'); 

            options = struct;
            options.ipopt.max_iter = 5000;
            opti.solver('ipopt', options); 
            optimizer = opti.to_function('f', {X0, del_min, X_tf, Y_tf}, {U, X, tf});
        end
    end
end