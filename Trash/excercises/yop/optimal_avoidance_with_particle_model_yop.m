clear all;
clc; 

load('opti_mov_yop.mat');

%% parameters and initial conditions
% Given values
v = 20; % m/s
% A = 12; % m

% initial positions
Ypos = 0; 
Xpos = 0; 
% Xend = ; %  calculated from slide 25 lecture 5 

% parameters
params.m = 2000;
params.g = 9.81;
params.mu = 0.6; % wet asphalt 

% obstacle
% calculated from slide 25 lecture 5 
obst.c = 6.35+34+2; % center (calculated from A = 34, and w_car + C)
obst.w = 6.35+2; % width (w_car + C)
obst.h = 1 + Ypos; % height (wcar/2 + max(8.5,x), x is any real number)
obst.n = 6; % sharpness 

%% YOP 
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: xpos ypos vx vy % position, speed
yops Control: u1 u2 % acceleration

x = [xpos; ypos; vx; vy];
u = [u1; u2];

% optimal control problem
ocp = yop.ocp('optimal_avoidance_with_particle_model');

% constrains
umax = params.mu*params.g*params.m;
ocp.st((u1.^2 + u2.^2) <= umax^2);
% ocp.st(u1 <= 0);
ocp.st(u2 >= 0);
ocp.st(der(x) == smp_veh_mdl(x,u,params)); 
ocp.st(xpos(t0) <= xpos <= xpos(tf));
ocp.st(ypos <= (obst.h + 3));
ocp.st(ypos >= 0);
% ocp.st(vx >= 0);
% ocp.st(vx >= 0);
% 
% obstacle
ocp.st(obstacle(xpos,ypos,obst) >= 1);

% initial conditrions
ocp.st(0 == t0 <= tf); 
ocp.st(x(t0) == [Xpos; Ypos; v; 0]);
ocp.st(xpos(tf) == 2*obst.c); 
ocp.st(ypos(tf) == obst.h + 2);

% cost function 
ocp.min(tf);

% solving 
sol = ocp.solve;
% sol = ocp.solve('guess',sol);

%% plottings

figure(1); clf;
tiledlayout('flow');

nexttile;
sol.plot(xpos,ypos); hold on;
xlabel('$x$ [m]','Interpreter','latex');
ylabel('$y$ [m]','Interpreter','latex');
fimplicit(@(x,y) obstacle(x, y, obst) - 1,[0 2*obst.c 0 1.1*obst.h],'Color','k');

nexttile;
sol.plot(t,vx);
ylabel('$v_x$ [m/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
sol.plot(t,vy);
ylabel('$v_y$ [m/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
sol.plot(t,sqrt(vx^2 + vy^2));
ylabel('$V_s$ [m/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
sol.plot(t,u1);
yline(-umax,'--','LineWidth',2);
ylabel('$u_1$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
sol.plot(t,u2);
yline(umax,'--','LineWidth',2);
ylabel('$u_2$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')
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


% function for the obstacle 
function val = obstacle(x,y,obst)

% parameters
c = obst.c;
h = obst.h;
w = obst.w;
n = obst.n;

val = ((x - c)/w).^n + (y/h).^n;
end