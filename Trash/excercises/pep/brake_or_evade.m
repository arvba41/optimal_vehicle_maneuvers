clear all; clc;
%% Problem: Double lane change, ISO-3888-2 (Älgtest, moose test, elk test)
% Consider driving at 20 m/s (72 km/h).
% For wet asphalt, µ = 0.6, the braking distance is 34 m.

% Given values
v = 20; % m/s
A = 12; % m

%% parameters
params.m = 2000;
params.g = 9.81;

Owdt = 11;
Ocnt = A + 13.5 + Owdt/2;

Ypos = 0;
%% Calculating A (minimum required friction to avoid large obstacles)
% Dry asphalt
params.mu = 1;

% OCP 
N = 100;                 % number of control intervals
opti = casadi.Opti();   % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
xpos = X(1,:);
ypos = X(2,:);
vx = X(3,:);
vy = X(4,:);
U = opti.variable(2,N);   % control trajectory (wheel force)
u1 = U(1,:);
u2 = U(2,:);
T = opti.variable();      % final time

vfin = vx(end)^2 + vy(end)^2;
opti.minimize(T);         % minimize exit speed

% ---- integration method ----
dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration  
   k1 = smp_veh_mdl(X(:,k),         U(:,k), params);
   k2 = smp_veh_mdl(X(:,k)+dt/2*k1, U(:,k), params);
   k3 = smp_veh_mdl(X(:,k)+dt/2*k2, U(:,k), params);
   k4 = smp_veh_mdl(X(:,k)+dt*k3,   U(:,k), params);
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- constrains ----
opti.subject_to((u1.^2 + u2.^2) <= (params.mu*params.m*params.g)^2);
opti.subject_to(T >= 0);
opti.subject_to(u1(1) <= 0);
opti.subject_to(u1(2) <= 0);

% ---- initial conditions ----
opti.subject_to(xpos(1) == 0);
opti.subject_to(ypos(1) == Ypos);
opti.subject_to(ypos(end) == ypos(1));
opti.subject_to(vx(1) == v);
opti.subject_to(vfin == 0);
opti.subject_to(vy(1) == 0);
opti.subject_to(u1(1) == 0);
opti.subject_to(u1(2) == 0);

% ---- inital guess ----
opti.set_initial(u1, -(params.mu*params.m*params.g));
opti.set_initial(T, 1);

% ---- solve NLP ------
p_opts = struct('expand',true);
s_opts = struct('print_level',0);
opti.solver('ipopt',p_opts,s_opts);
sol = opti.solve();   % actual solve

%%% ---- Analytical ----
mu = params.mu; % road friction 
g = params.g;
A_large_obstacles = 1/2* v^2/(mu*g);
% Displaying the results
disp(['Analytical: Minimum required friction to avoid large obstacles (A = ', num2str(A_large_obstacles), '): µ = ', num2str(mu)]);
disp(['Simulation: Minimum required friction to avoid large obstacles (A = ', num2str(sol.value(xpos(end))), '): µ = ', num2str(params.mu)]);

%% Calculating A (minimum required friction to avoid large obstacles)
% Wet asphalt
params.mu = 0.6;

% OCP 
N = 100;                 % number of control intervals
opti = casadi.Opti();   % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
xpos = X(1,:);
ypos = X(2,:);
vx = X(3,:);
vy = X(4,:);
U = opti.variable(2,N);   % control trajectory (wheel force)
u1 = U(1,:);
u2 = U(2,:);
T = opti.variable();      % final time

vfin = vx(end)^2 + vy(end)^2;
opti.minimize(T);         % minimize exit speed

% ---- integration method ----
dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration  
   k1 = smp_veh_mdl(X(:,k),         U(:,k), params);
   k2 = smp_veh_mdl(X(:,k)+dt/2*k1, U(:,k), params);
   k3 = smp_veh_mdl(X(:,k)+dt/2*k2, U(:,k), params);
   k4 = smp_veh_mdl(X(:,k)+dt*k3,   U(:,k), params);
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- constrains ----
opti.subject_to((u1.^2 + u2.^2) <= (params.mu*params.m*params.g)^2);
opti.subject_to(T >= 0);
opti.subject_to(u1(1) <= 0);
opti.subject_to(u1(2) <= 0);

% ---- initial conditions ----
opti.subject_to(xpos(1) == 0);
opti.subject_to(ypos(1) == Ypos);
opti.subject_to(ypos(end) == ypos(1));
opti.subject_to(vx(1) == v);
opti.subject_to(vfin == 0);
opti.subject_to(vy(1) == 0);
opti.subject_to(u1(1) == 0);
opti.subject_to(u1(2) == 0);

% ---- inital guess ----
opti.set_initial(u1, -(params.mu*params.m*params.g));
opti.set_initial(T, 1);

% ---- solve NLP ------
p_opts = struct('expand',true);
s_opts = struct('print_level',0);
opti.solver('ipopt',p_opts,s_opts);
sol = opti.solve();   % actual solve

%%% ---- Analytical ----
mu = params.mu; % road friction 
g = params.g;
A_large_obstacles = 1/2* v^2/(mu*g);
% Displaying the results
disp(['Analytical: Minimum required friction to avoid large obstacles (A = ', num2str(A_large_obstacles), '): µ = ', num2str(mu)]);
disp(['Simulation: Minimum required friction to avoid large obstacles (A = ', num2str(sol.value(xpos(end))), '): µ = ', num2str(params.mu)]);

%% Calculating A (minimum required friction to avoid large obstacles)
% Snow/ice asphalt
params.mu = 0.3;

% OCP 
N = 500;                 % number of control intervals
opti = casadi.Opti();   % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
xpos = X(1,:);
ypos = X(2,:);
vx = X(3,:);
vy = X(4,:);
U = opti.variable(2,N);   % control trajectory (wheel force)
u1 = U(1,:);
u2 = U(2,:);
T = opti.variable();      % final time

vfin = vx(end)^2 + vy(end)^2;
opti.minimize(T);         % minimize exit speed

% ---- integration method ----
dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration  
   k1 = smp_veh_mdl(X(:,k),         U(:,k), params);
   k2 = smp_veh_mdl(X(:,k)+dt/2*k1, U(:,k), params);
   k3 = smp_veh_mdl(X(:,k)+dt/2*k2, U(:,k), params);
   k4 = smp_veh_mdl(X(:,k)+dt*k3,   U(:,k), params);
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- constrains ----
opti.subject_to((u1.^2 + u2.^2) <= (params.mu*params.m*params.g)^2);
opti.subject_to(T >= 0);
opti.subject_to(u1(1) <= 0);
opti.subject_to(u1(2) <= 0);

% ---- initial conditions ----
opti.subject_to(xpos(1) == 0);
opti.subject_to(ypos(1) == Ypos);
opti.subject_to(ypos(end) == ypos(1));
opti.subject_to(vx(1) == v);
opti.subject_to(vfin == 0);
opti.subject_to(vy(1) == 0);
opti.subject_to(u1(1) == 0);
opti.subject_to(u1(2) == 0);

% ---- inital guess ----
opti.set_initial(u1, -(params.mu*params.m*params.g));
opti.set_initial(T, 1);

% ---- solve NLP ------
p_opts = struct('expand',true);
s_opts = struct('print_level',0);
% opti.solver('ipopt',p_opts,s_opts);
opti.solver('ipopt');
sol = opti.solve();   % actual solve

%%% ---- Analytical ----
mu = params.mu; % road friction 
g = params.g;
A_large_obstacles = 1/2* v^2/(mu*g);
% Displaying the results
disp(['Analytical: Minimum required friction to avoid large obstacles (A = ', num2str(A_large_obstacles), '): µ = ', num2str(mu)]);
disp(['Simulation: Minimum required friction to avoid large obstacles (A = ', num2str(sol.value(xpos(end))), '): µ = ', num2str(params.mu)]);

%% Calculating B (minimum required friction to avoid large obstacles)
% Wet asphalt
A = 25.5; 
params.mu = 0.6;

% OCP 
N = 100;                 % number of control intervals
opti = casadi.Opti();   % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
xpos = X(1,:);
ypos = X(2,:);
vx = X(3,:);
vy = X(4,:);
U = opti.variable(2,N);   % control trajectory (wheel force)
u1 = U(1,:);
u2 = U(2,:);
T = opti.variable();      % final time

vfin = vx(end)^2 + vy(end)^2;
opti.minimize(T);         % minimize exit speed

% ---- integration method ----
dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration  
   k1 = smp_veh_mdl(X(:,k),         U(:,k), params);
   k2 = smp_veh_mdl(X(:,k)+dt/2*k1, U(:,k), params);
   k3 = smp_veh_mdl(X(:,k)+dt/2*k2, U(:,k), params);
   k4 = smp_veh_mdl(X(:,k)+dt*k3,   U(:,k), params);
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- constrains ----
opti.subject_to((u1.^2 + u2.^2) <= (params.mu*params.m*params.g)^2);
opti.subject_to(T >= 0);
opti.subject_to(u1(1) <= 0);
opti.subject_to(u1(2) <= 0);
opti.subject_to(vx >= 0);

% ---- initial conditions ----
opti.subject_to(xpos(1) == 0);
opti.subject_to(ypos(1) == Ypos);
opti.subject_to(xpos(end) == A);
opti.subject_to(ypos(end) >= ypos(1));
% opti.subject_to(xpos(1) <= xpos <= xpos(end));
opti.subject_to(vx(1) == v);
opti.subject_to(vfin == 0);
opti.subject_to(vy(1) == 0);
opti.subject_to(u1(1) == 0);
opti.subject_to(u1(2) == 0);

% ---- inital guess ----
opti.set_initial(u1, -(params.mu*params.m*params.g));
opti.set_initial(T, 1);

% ---- solve NLP ------
% p_opts = struct('expand',true);
% s_opts = struct('print_level',0);
% opti.solver('ipopt',p_opts,s_opts);
opti.solver('ipopt');
sol = opti.solve();   % actual solve

%% simulation 
A = 34; 
params.mu = 0.6;

% u = @(t) -[(params.mu*params.g*params.m); 0];
u = @(t) -[1; -8.5/25.5]*sqrt((params.mu*params.m*params.g)^2/(1+(8.5/25.5)^2));

tf = 3.6;
ts = 0;

N = 100;

[t,y] = ode45(@(t,y) smp_veh_mdl(y,u(t),params),[ts tf],[0; 0; 20; 0]);
% smp_veh_mdl([0; 0; 20; 0],u(1),params)

%% PLOTS
figure(1); clf

tiledlayout('flow');

nexttile;
plot(y(:,1),y(:,2))
title('tragectory')

nexttile;
plot(t,y(:,3))
title('vx')

nexttile;
plot(t,y(:,4))
title('vy')

% nexttile;
% plot(opti.debug.value(u1))
% title('u1')

nexttile;
plot(t,u(t))
title('u2')

%% PLOTS
figure(1); clf

tiledlayout('flow');

nexttile;
plot(opti.debug.value(xpos),opti.debug.value(ypos))
title('tragectory')

nexttile;
plot(opti.debug.value(vx))
title('vx')

nexttile;
plot(opti.debug.value(vy))
title('vy')

nexttile;
plot(opti.debug.value(u1))
title('u1')

nexttile;
plot(opti.debug.value(u2))
title('u2')

%%% ---- Analytical ----
mu = params.mu; % road friction 
g = params.g;
B_large_obstacles = mu * 9.81 * (A/v)^2 / 2;
% Displaying the results
disp(['Analytical: Minimum required friction to avoid large obstacles (B = ', num2str(B_large_obstacles), '): µ = ', num2str(mu)]);
disp(['Simulation: Minimum required friction to avoid large obstacles (B = ', num2str(sol.value(ypos(end))), '): µ = ', num2str(params.mu)]);
%% Calculating minimum required friction to evade with B = 1.7
B_required = 1.7;
mu_required = sqrt(2 * B_required / (mu * 9.81)) * v / A;

% Calculating A for a close touch even with thinking time
B = 1.7;
A_close_touch = mu * 9.81 * (B * v^2 / (2 * mu))^0.5;

% Displaying the results
disp(['Minimum required friction to avoid large obstacles (B = ', num2str(A_large_obstacles), '): µ = ', num2str(mu)]);
disp(['Minimum required friction to evade with B = 1.7: µ = ', num2str(mu_required)]);
disp(['Braking distance for a close touch even with thinking time (B = ', num2str(B), '): A = ', num2str(A_close_touch)]);


%% functions

% using a simple vehcile model 
function [x_dot] = smp_veh_mdl(x,u,params)

% states
% xpos = x(1);
vx = x(3);
vy = x(4);

% inputs
u1 = u(1);
u2 = u(2);

% parameters 
m = params.m;
% mu = params.mu;

% equations
xpos_dot = vx;
ypos_dot = vy;
vx_dot = u1/m;
vy_dot = u2/m;

x_dot = [xpos_dot; ypos_dot; vx_dot; vy_dot];
end

