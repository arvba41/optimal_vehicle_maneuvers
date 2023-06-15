clear all; clc;
%% YOP 
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: v h m nominal: [1500 1e5 100]   % position, speed
yops Control: F  nominal: [10]   % acceleration
% yops Param: l       % maximum cart position

% t = yop.time; t0 = yop.time0; tf = yop.timef;


% states
% v = yop.state; 
% h = yop.state;
% m = yop.state;

x = [v; h; m];

% F = yop.control;

% optimal control problem
ocp = yop.ocp('goddard_rocket');

% maximize the height
ocp.max(int(v));

% parameters
params.D0 = 0.01227;
params.g0 = 9.81;
params.beta = 0.145e-3;
params.r0 = 6.371e6;
params.c = 2060;

% constraints
ocp.st(0 <= F <= 9.525515); % force constrains
ocp.st(der(x) == rockt_dyn(x,F,params)); 

% initial conditrions
ocp.st(0 == t0 <= tf); % set initial time and final time > init time
ocp.st(x(t0) == [0; 0; 214.839]);
ocp.st(67.9833 <= m <= 214.839);
ocp.st(0 <= [h; v]);
ocp.st(der(h(t < 0.25)) <= 300);
ocp.st(v(tf) == 0);
% solving 
sol = ocp.solve;

%% plots 
figure(1); 

subplot(121)
sol.plot(t,h); ylabel('h [m]'); xlabel('t [s]'); hold on

subplot(122);
sol.plot(t,v); ylabel('v [km/h]'); xlabel('t [s]'); hold on


%% rocket dynamics
function [x_dot] = rockt_dyn(x,u,params)

% ---- parameters ----
D0  = params.D0;
g0 = params.g0;
beta = params.beta;
r0 = params.r0;
c = params.c;

% ---- inputs 
F = u;

% ---- states ----
v = x(1);
h = x(2);
m = x(3);

% ---- functions ----
D = D0*v^2*exp(-beta*h); % function for drag
g = g0*(r0/(r0 + h))^2; % function for gravitation acceleration

% --- ODEs ----
v_dot = (u*c - D)/m - g;
h_dot = v;
m_dot = -F;

x_dot = [v_dot; h_dot; m_dot];
% y = 0;
end