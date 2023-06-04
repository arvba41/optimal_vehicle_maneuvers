clear all; clc;
%% Problem: Double lane change, ISO-3888-2 (Älgtest, moose test, elk test)
% Consider driving at 20 m/s (72 km/h).
% For wet asphalt, µ = 0.6, the braking distance is 34 m.

% Given values
v = 20; % m/s
A = 12; % m

% initial positions
Ypos = 0; 
Xpos = 0; 

% parameters
params.m = 2000;
params.g = 9.81;

%% Optimization problem: minimum longitudanal distance required to avoid obstacles
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: xpos ypos vx vy % position, speed
yops Control: u1 u2 % acceleration

B = 1.7;
params.mu = 0.6; % friction for wet asphalt 

x = [xpos; ypos; vx; vy];
u = [u1; u2];

% optimal control problem
ocp = yop.ocp('Wet_asphalt_evading_distance');

% maximize the height
ocp.min(xpos(tf));

% constraints
umax = params.mu*params.g*params.m;
ocp.st(u1 == 0);
ocp.st(0 <= u2 <= umax); 
ocp.st(der(x) == smp_veh_mdl(x,u,params)); 

% initial conditrions
ocp.st(0 == t0 <= tf); 
ocp.st(x(t0) == [0; 0; 20; 0]);
ocp.st(ypos(tf) == B);

% solving 
sol = ocp.solve;

% ---- Analytical ----
results.sim.Yp1 = sol.value(ypos);
results.params.mu4 = params.mu; % road friction 
mu = params.mu; % road friction 
g = params.g;
results.sim.B1 = mu * g * (A/v)^2 / 2;

% ---- plots ----
% verification plots
figure(10); clf; 
subplot(3,2,1:2)
sol.plot(xpos,ypos); ylabel('y [m]'); xlabel('x [m]');

subplot(3,2,3)
sol.plot(t,vx); ylabel('v_x [m/s]'); xlabel('t [s]');

subplot(3,2,4)
sol.plot(t,vy); ylabel('v_y [m/s]'); xlabel('t [s]');

subplot(3,2,5)
yyaxis left
sol.plot(t,u1/1e3); ylabel('F_x [kN]'); xlabel('t [s]');
yyaxis right
sol.plot(t,u2/1e3); ylabel('F_y [kN]'); xlabel('t [s]');

subplot(3,2,6)
sol.plot(t,sqrt(u1.^2 + u2.^2)/1e3); ylabel('F [kN]'); xlabel('t [s]');
yline(umax/1e3,'--');

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
