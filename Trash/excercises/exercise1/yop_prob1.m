clear all; clc;
%% the optimization problem parameterization
% parameters
params.m = 500;
params.g = 9.81;
params.mu = 0.5;

% conditions
Xsrt = 0;
Ysrt = 1;
Xa = 50;
Xfin = 2*Xa;
Yfin = 1;
Vxsrt = 0/3.6;
Vysrt = 0;

% obstacle
obs.Xa = Xa;
obs.R1 = 2;
obs.R2 = 1.5;
obs.pow = 6;


%% the optimization problem itself
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: xpos ypos vx vy % position, speed
yops Control: u1 u2 % acceleration

x = [xpos; ypos; vx; vy];
u = [u1; u2];

% optimal control problem
ocp = yop.ocp('obt_aviod_friction_limited_particle');

% parameters
m = params.m;
g = params.g;
mu = params.mu;

% maximize the height
ocp.min(tf)

% constraints
ocp.st((u1^2 + u2^2)<= (mu*m*g)^2); % force constrains
ocp.st(der(x) == friction_limited_particle(x,u,params)); 

% obstacle
Xa = obs.Xa;
R1 = obs.R1;
R2 = obs.R2;
pow = obs.pow;
ocp.st((-((xpos-Xa)/R1)^pow -(ypos/R2)^pow) + 1 <= 0)

% initial conditrions
ocp.st(0 == t0 <= tf); % set initial time and final time > init time
ocp.st(xpos(t0) == Xsrt);
ocp.st(ypos(t0) == Ysrt);
ocp.st(vx(t0) == Vxsrt);
ocp.st(vy(t0) == Vysrt);

% boundery conditions
ocp.st(xpos(tf) == Xfin);
ocp.st(ypos(tf) == Yfin);

% solving 
sol = ocp.solve('ival',40);

%% plots 
figure(10); clf;

sol.plot(xpos,ypos); 

% subplot(122);
% sol.plot(t,v); ylabel('v [km/h]'); xlabel('t [s]'); hold on

%% friction limited particle fucntion 
function [x_dot] = friction_limited_particle(x,u,params)

% parameters
m = params.m;

% states
xpos = x(1);
ypos = x(2);
vx = x(3);
vy = x(4);

% inputs
u1 = u(1);
u2 = u(2);

% ---- ODEs ----
xpos_dot = vx;
ypos_dot = vy;
vx_dot = u1/m;
vy_dot = u2/m;

x_dot = [xpos_dot; ypos_dot; vx_dot; vy_dot];

end


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