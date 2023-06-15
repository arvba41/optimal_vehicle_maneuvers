clear all; clc;
%% ---- model parameterization ----
% Pacejka Magic Formula with Friction Elipse parameters
params.mux = [1.2 1.2];
params.Bx = [11.7 11.1];
params.Cx = [1.69 1.69];
params.Ex = [0.377 0.362];
params.muy = [0.935 0.961];
params.By = [8.86 9.30];
params.Cy = [1.19 1.19];
params.Ey = [-1.21 -1.11];

% ST model parameters
params.m = 2100;
params.lf = 1.3; 
params.lr = 1.5;
params.g = 9.82;
params.Izz = 3900;
params.Iw = 4;
params.Re = 0.3;

Vstrt= 20/3.6;
Xfin = 100/2;
%% Car Avoiding an obstacle
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

% ST model parameters
m = params.m;
lf = params.lf; 
lr = params.lr;
g = params.g;
Izz =params.Izz;
Iw = params.Iw;
Re = params.Re;

N = 40; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(7,N+1); % state trajectory
xpos    = X(1,:);
ypos    = X(2,:);
vx      = X(3,:);
vy      = X(4,:);
r       = X(5,:);
wf      = X(6,:);
wr      = X(7,:);
U = opti.variable(2,N);   % control trajectory (steering and throttle)
delta   = U(1,:);
Tf      = U(2,:);
T = opti.variable();      % final time

% ---- objective          ---------
opti.minimize(T);         % minimize time

% ---- dynamic constraints --------
% ---- intermediate equations ----
F_X = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,1);
F_Y = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,2);
M_Z = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,3);

Fx_f = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,4);
Fx_r = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,6);
% ---- ODE ----
f = @(x,u) [ ... the posistion dynamics ...
                    x(3);...                    \dot x = vx
                    x(4);...                    \dot y = vy
             ... the speed dynamics ...
                    F_X(x(3),x(4),x(5),x(6),x(7),u(1))/m + x(4)*x(5);...  \dot vx = FX/m + vy*r
                    F_Y(x(3),x(4),x(5),x(6),x(7),u(1))/m - x(3)*x(5);...  \dot vx = FY/m - vx*r
                    M_Z(x(3),x(4),x(5),x(6),x(7),u(1))/Izz;...            \dot yawrate = MZ/Izz
             ... wheel dynamics ...
                    (u(2) - Re*Fx_f(x(3),x(4),x(5),x(6),x(7),u(1)))/Iw;...\dot omegaf = (Tf - Re*Fxf)/Iw
                    -Re*Fx_r(x(3),x(4),x(5),x(6),x(7),u(1))/Iw...         \dot omegar = (- Re*Fxr)/Iw
                    ]; 

dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- force constraints -----------
% Friction elipse
[~,miscvars] = combined_forces_FXFY(params,vx(2:end),vy(2:end),r(2:end),wf(2:end),wr(2:end),delta,6);
Fx_f = miscvars.Fxf;
Fy_f = miscvars.Fxr;
Fx_r = miscvars.Fyf;
Fy_r = miscvars.Fyr;

Fflim = Fx_f.^2 + Fy_f.^2;
% muxf = params.mux(1); muxr = params.mux(2); 
% muyf = params.muy(2); muyr = params.muy(2);
Fzfmax = m*g*lr/(lf+lr);
% Flim = U(1,:).^2 +  U(2,:).^2; % Force limit equation
opti.subject_to(Fflim<=Fzfmax^2); % Force is limited eliptically

opti.subject_to(Tf<=3000); % Force is limited eliptically

% ---- boundary conditions --------
% define all the initial values here 
opti.subject_to(xpos(1)==0);        % start at position x 0 ...
opti.subject_to(ypos(1)==1);        % start at position y 1 ... 
opti.subject_to(xpos(N+1)==2*Xfin); % finish line at position x ...
opti.subject_to(ypos(N+1)==1);      % finish line at position y ...
opti.subject_to(vx(1)==Vstrt);     % start with speed x ...
opti.subject_to(vy(1)==0);          % start with speed y ...
opti.subject_to(r(1)==0);           % start with yaw rate ...
opti.subject_to(wf(1)==0);          % start with angular velocity font ...
opti.subject_to(wr(1)==0);          % start with angular velocity rear ...

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
% define all the initial values here 
opti.set_initial(xpos, 0);
% opti.set_initial(ypos, 1);
% opti.set_initial(xspeed, 1);
% opti.set_initial(yspeed, 1);
opti.set_initial(T, 1);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

%% converting to readable values
prob1.x = sol.value(xpos); prob1.y = sol.value(ypos);
prob1.vx = sol.value(xspeed); prob1.vy = sol.value(yspeed);
prob1.ux = sol.value(U(1,:)); prob1.uy = sol.value(U(2,:));
prob1.topt = sol.value(T); prob1.tvec = linspace(0,sol.value(T),N+1);

prob1.cons.Flim = mass*mu*g;
%% plotting
f = figure(1); clf;
optim_plot_prob1(prob1);

