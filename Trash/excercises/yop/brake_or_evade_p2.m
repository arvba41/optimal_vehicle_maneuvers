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

% Xend = 20.3; % stopping distance (DRY asphalt)
% Xend = 34; % stopping distance (WET asphalt)
Xend = 68; % stopping distance (ICE asphalt)

% parameters
params.m = 2000;
params.g = 9.81;

%% Optimization problem: minimum lateral distance required to avoid obstacles
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: xpos ypos vx vy % position, speed
yops Control: u1 u2 % acceleration

A = 34;
params.mu = 0.6; % friction for wet asphalt 

x = [xpos; ypos; vx; vy];
u = [u1; u2];

% optimal control problem
ocp = yop.ocp('Wet_asphalt_evading_distance');

% maximize the height
% ocp.min(tf)
ocp.max(ypos(tf));

% constraints
umax = params.mu*params.g*params.m;
ocp.st(u1 == 0);
ocp.st(u2 == umax); 
ocp.st(der(x) == smp_veh_mdl(x,u,params)); 

% initial conditrions
ocp.st(0 == t0 <= tf); 
ocp.st(x(t0) == [0; 0; 20; 0]);
ocp.st(xpos(tf) == A);

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
% umax = sol.value(mu)*params.g*params.m;
yline(umax/1e3,'--');

%% Optimization problem: minimum required friction to avoid obstacles
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: xpos ypos vx vy % position, speed
yops Control: u1 u2 % acceleration
yops Parameters: mu;

A = 34; B = 1.7;
% params.mu = 0.6; % friction for wet asphalt 

x = [xpos; ypos; vx; vy];
u = [u1; u2];

% optimal control problem
ocp = yop.ocp('evading_minimum_required_friction');

% maximize the height
ocp.min(mu);
% ocp.max(ypos(tf));

% constraints
umax = mu*params.g*params.m;
ocp.st(u1 == 0);
ocp.st(0 <= u2 <= umax); 
ocp.st(der(x) == smp_veh_mdl(x,u,params)); 

% initial conditrions
ocp.st(0 == t0 <= tf); 
ocp.st(x(t0) == [0; 0; 20; 0]);
ocp.st(xpos(tf) == A);
ocp.st(ypos(tf) == B);

% solving 
sol = ocp.solve;

% ---- plots ----
% verification plots
figure(11); clf; 
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
umax = sol.value(mu)*params.g*params.m;
yline(umax/1e3,'--');

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
% ocp.min(tf)
ocp.min(xpos(tf));

% constraints
umax = params.mu*params.g*params.m;
ocp.st(u1 == 0);
ocp.st(u2 == umax); 
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
figure(12); clf; 
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
% umax = sol.value(mu)*params.g*params.m;
yline(umax/1e3,'--');

%% Optimization problem 5: minimum lateral distance required to avoid obstacles
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: xpos ypos vx vy % position, speed
yops Control: u1 u2 % acceleration

A = 34;
params.mu = 0.12; % friction for insanely slipary asphalt 

x = [xpos; ypos; vx; vy];
u = [u1; u2];

% optimal control problem
ocp = yop.ocp('Dry_asphalt_stopping_distance');

% maximize the height
ocp.min(tf)

% constraints
umax = params.mu*params.g*params.m;
ocp.st(u1 == 0);
ocp.st(0 <= u2 <= umax); 
ocp.st(der(x) == smp_veh_mdl(x,u,params)); 

% initial conditrions
ocp.st(0 == t0 <= tf); 
ocp.st(x(t0) == [0; 0; 20; 0]);
ocp.st(xpos(tf) == A);

% solving 
sol = ocp.solve;

% ---- Analytical ----
results.sim.Yp2 = sol.value(ypos);
results.params.mu5 = params.mu; % road friction 
mu = params.mu; % road friction 
g = params.g;
results.sim.B2 = mu * g * (A/v)^2 / 2;

%% Optimization problem 6: minimum lateral distance required to avoid obstacles
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: xpos ypos vx vy % position, speed
yops Control: u1 u2 % acceleration

% A = 34;
params.mu = 0.6; % friction for wet asphalt 

x = [xpos; ypos; vx; vy];
u = [u1; u2];

% optimal control problem
ocp = yop.ocp('Dry_asphalt_stopping_distance');

% maximize the height
ocp.min(tf)

% constraints
umax = params.mu*params.g*params.m;
ocp.st(u1 == 0);
ocp.st(u2 == umax); 
ocp.st(der(x) == smp_veh_mdl(x,u,params)); 

% initial conditrions
ocp.st(0 == t0 <= tf); 
ocp.st(x(t0) == [0; 0; 20; 0]);
ocp.st(ypos(tf) == 1.7);

% solving 
sol = ocp.solve;

% ---- Analytical ----
results.sim.Yp3 = sol.value(xpos);
results.params.mu6 = params.mu; % road friction 
mu = params.mu; % road friction 
g = params.g;
results.sim.B3 =v*sqrt(2*1.7/(mu*g));

%% display the results 
disp(['Analytical: Minimum required friction to avoid large obstacles (A = ', num2str(results.sim.A1), '): µ = ', num2str(results.params.mu1)]);
disp(['Simulation: Minimum required friction to avoid large obstacles (A = ', num2str(results.sim.Xp1(end)), '): µ = ', num2str(results.params.mu1)]);
disp('--------------------------');
disp(['Analytical: Minimum required friction to avoid large obstacles (A = ', num2str(results.sim.A2), '): µ = ', num2str(results.params.mu2)]);
disp(['Simulation: Minimum required friction to avoid large obstacles (A = ', num2str(results.sim.Xp2(end)), '): µ = ', num2str(results.params.mu2)]);
disp('--------------------------');
disp(['Analytical: Minimum required friction to avoid large obstacles (A = ', num2str(results.sim.A3), '): µ = ', num2str(results.params.mu3)]);
disp(['Simulation: Minimum required friction to avoid large obstacles (A = ', num2str(results.sim.Xp3(end)), '): µ = ', num2str(results.params.mu3)]);
disp('--------------------------');
disp(['Analytical: Braking distance for a close touch (B = ', num2str(results.sim.B1), '): µ = ', num2str(results.params.mu4)]);
disp(['Simulation: Braking distance for a close touch (B = ', num2str(results.sim.Yp1(end)), '): µ = ', num2str(results.params.mu4)]);
disp('--------------------------');
disp(['Analytical: Braking distance for a close touch (B = ', num2str(results.sim.B2), '): µ = ', num2str(results.params.mu5)]);
disp(['Simulation: Braking distance for a close touch (B = ', num2str(results.sim.Yp2(end)), '): µ = ', num2str(results.params.mu5)]);
disp('--------------------------');
disp(['Analytical: Braking distance for a close touch (A = ', num2str(results.sim.B3), '): µ = ', num2str(results.params.mu6)]);
disp(['Simulation: Braking distance for a close touch (A = ', num2str(results.sim.Yp3(end)), '): µ = ', num2str(results.params.mu6)]);

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
