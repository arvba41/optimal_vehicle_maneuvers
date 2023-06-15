clear all; clc;

%% optimization problem conditions 

% center point of the obstacle 
Xa = 50;

% starting and stoping distance 
Xsrt = 0;
Xfin = 2*Xa - Xsrt;
Ysrt = 1;
Yfin = 1;

Ylim = 5;

% initial velovity 
Vxsrt = 10/3.6;

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;

%%%%% Parameters 
% Pacejka Magic Formula parameters
params.mux = [1.2 1.2];
params.Bx = [11.7 11.1];
params.Cx = [1.69 1.69];
params.Ex = [0.377 0.362];
params.muy = [0.935 0.961];
params.By = [8.86 9.30];
params.Cy = [1.19 1.19];
params.Ey = [-1.21 -1.11];
% vehicle parameters
params.m = 2100;
params.lf = 1.3; 
params.lr = 1.5;
params.g = 9.82;
params.Izz = 3900; 
params.Iw = 4;
params.Re = 0.3;
% input time constants
params.TdFxf = 0.2;
params.TdFxr = 0.2;

% model test
[x_dot] = single_track([0;0;10/3.6;0;0;0;0;0;0],[0;-100;0],params)

%% optimization problem itself

yops Times: t t0 tf 
yops States: x y vx vy r delta psi Fxf Fxr
yops Control: ddelta Fxf_in Fxr_in

s = [x; y; vx; vy; r; delta; psi; Fxf; Fxr]; % state vector
u = [ddelta; Fxf_in; Fxr_in]; % input vector

%%% ---- optimal control problem
ocp = yop.ocp('ST_avoid_obstacle');

ocp.min(tf); % cost function

% constraintrs
ocp.st(-ddeltamax <= ddelta <= ddeltamax);
ocp.st(-deltamax <= delta <= deltamax);

Fzf = params.m*params.g*params.lr/(params.lf+params.lr); 
Fzr = params.m*params.g*params.lf/(params.lf+params.lr);
Dxf = params.mux(1).*Fzf; Dxr = params.mux(2).*Fzr;
ocp.st(-Dxf <= Fxf <= Dxf);
ocp.st(-Dxr <= Fxr <= 0);
ocp.st(Fxf(t0) == 0);
ocp.st(Fxr(t0) == 0);

% dynamic constraints
ocp.st(der(s) == single_track(s,u,params))

% initial conditions
ocp.st(0 == t0 <= tf);  % time
ocp.st(x(t0) == Xsrt);
ocp.st(x(tf) == Xfin);
ocp.st(y(t0) == Ysrt);
ocp.st(y(tf) == Yfin);
ocp.st(vx(t0) == Vxsrt);
ocp.st(vx >= 0);
% ocp.st(vy(t0) == 0);
% ocp.st(0 <= x <= Xfin)
% ocp.st(0 <= y <= Ylim)

% solving 
sol = ocp.solve('ival',10);

%% plottings

figure(10); clf;

sol.plot(x,y);

figure(11); clf;
tiledlayout('flow');

nexttile;
sol.plot(t,vx);

nexttile;
sol.plot(t,vy);

nexttile;
sol.plot(t,delta);

nexttile;
sol.plot(t,ddelta);

nexttile;
sol.plot(t,Fxf);

nexttile;
sol.plot(t,Fxr);

%% single track model 
function [x_dot] = single_track(x,u,params)

% ST model parameters
m   = params.m;
lf  = params.lf; 
lr  = params.lr;
g   = params.g;
Izz = params.Izz;
% Iw  = params.Iw;
% Re  = params.Re;

% friction paramteres
muxf = params.mux(1); muxr = params.mux(2); 
muyf = params.muy(2); muyr = params.muy(2);

% Pacejka parameters
Cxf = params.Cx(1); Cxr = params.Cx(2);
Cyf = params.Cy(1); Cyr = params.Cy(2);
Bxf = params.Bx(1); Bxr = params.Bx(2);
Byf = params.By(1); Byr = params.By(2);
Exf = params.Ex(1); Exr = params.Ex(2);
Eyf = params.Ey(1); Eyr = params.Ey(2);                 

% input filter parameters
TdFxf = params.TdFxf;
TdFxr = params.TdFxr;

%%% ---- states
xx = x(1);
yy = x(2);
vx = x(3);
vy = x(4);
r = x(5);
delta = x(6);
psi = x(7);
Fxf = x(8);
Fxr = x(9);

%%% ---- inputs
ddelta = u(1);
Fxf_in = u(2);
Fxr_in = u(3);

%%% ---- equations -----
% ---- independent wheel velocities ----
vxf = vx.*cos(delta)            + sin(delta).*(vy + lf.*r); 
vyf = cos(delta).*(vy + lf.*r)  - vx.*sin(delta);
vxr = vx;
vyr = vy + lr.*r;

% ---- determining the lateral slips ----
alphaf = -atan(vyf./vxf);
alphar = -atan(vyr./vxr);

% ---- determining the lateral forces on the wheels from Fy ----
% maximum forces
Fzf = m*g*lr/(lf+lr); 
Fzr = m*g*lf/(lf+lr);
Dxf = muxf.*Fzf; Dxr = muxr.*Fzr;
Dyf = muyf.*Fzf; Dyr = muyr.*Fzr;

Fy0f = Dyf.*sin(Cyf.*atan(Byf.*alphaf-Eyf.*(Byf.*alphaf - atan(Byf.*alphaf))));
Fy0r = Dyr.*sin(Cyr.*atan(Byr.*alphar-Eyr.*(Byr.*alphar - atan(Byr.*alphar))));
Fyf = Fy0f.*sqrt(1 - (Fxf/Dxf).^2);
Fyr = Fy0r.*sqrt(1 - (Fxr/Dxr).^2);

% ---- the total forces and moments acting on the vehcile ----
F_X = cos(delta).*Fxf - sin(delta).*Fyf + Fxr;  
F_Y = cos(delta).*Fyf + sin(delta).*Fxf + Fyr;
M_Z = lf*cos(delta).*Fyf + lf*sin(delta).*Fxf - lr*Fyr;

% ---- ODE ----
xx_dot = vx*cos(psi) - vy*sin(psi); 
yy_dot = vx*sin(psi) + vy*cos(psi); 

vx_dot = F_X/m + vy*r;
vy_dot = F_Y/m - vx*r;
r_dot = M_Z/Izz;

delta_dot = ddelta;

psi_dot = r;

Fxf_dot = (Fxf_in - Fxf)/TdFxf;
Fxr_dot = (Fxr_in - Fxr)/TdFxr;

x_dot = [xx_dot; yy_dot; vx_dot; vy_dot; r_dot; delta_dot; psi_dot; Fxf_dot; Fxr_dot];
end
