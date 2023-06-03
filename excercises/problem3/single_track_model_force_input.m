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

% ---- vehicle parameters ----
% ---- parameters ----
    m = params.m; % mass
    lf = params.lf; % CoM to front wheel center
    lr = params.lr; % CoM to rear wheel center
    Re = params.Re; % effective tire radius
    Izz = params.Izz; % yaw inertia
    Iw = params.Iw; % wheel inertial 
    g = params.g; % acceleration due to gravity 
    
% magic_formula_friction_elipse; %% look inside at your own risk
simplified_friction_elipse;
% ---- states ----
% x(1) -> x
% x(2) -> y
% x(3) -> vx
% x(4) -> vy
% x(5) -> yawrate (or \dot psi)
% ---- inputs ----
% u(1) --> delta 
% u(2) --> Fx (force in the x direction, at the front wheel)
% ---- ODEs ----
f = @(x,u) [ ... the posistion dynamics ...
                    x(3);...                    \dot x = vx
                    x(4);...                    \dot y = vy
                ... the speed dynamics ...
                    FX(x(3),x(4),x(5),u(1),u(2))/m + x(4)*x(5);...  \dot vx = FX/m + vy*yawrate
                    FY(x(3),x(4),x(5),u(1),u(2))/m - x(3)*x(5);...  \dot vx = FY/m - vx*yawrate
                    MZ(x(3),x(4),x(5),u(1),u(2))/Izz;...           \dot yawrate = MZ/Izz
                    ]; 

%% Simulating the model
%% --- simulation number 1 ---
% No steering angle only positive wheel torque
Fxfmax = 1e3; % maximum force
delta_max = 0; % maximum yawv rate

% ---- input functions ----
Hsigmoid = @(x) 0.5*(1+tanh(2*pi*6*(x)/(10)));
Ppulse = @(x,puls_strt,puls_end) Hsigmoid(x-puls_strt) + Hsigmoid(puls_end-x) - 1;

Tin = @(x,puls_strt,puls_end) Fxfmax*Ppulse(x,puls_strt,puls_end);
deltain = @(x,puls_strt,puls_end) cos(2*pi*(0)/10).*Ppulse(x,puls_strt,puls_end)*delta_max;

D_strt = 20;
D_end = 30;

Fxf_strt = 10;
Fxf_end = 40;

% ---- input test plots
% x = linspace(0,50,500);
% figure(2); clf;
% subplot(211);
% plot(x,Tin(x,T_strt,T_end),'LineWidth',2); box off
% subplot(212);
% plot(x,deltain(x,D_strt,D_end),'LineWidth',2); box off
% %%
tspan = [0 50];
y0 = zeros(5,1);
y0(3) = 0.1; y0(4) = 0;
[t,y] = ode15s(@(t,y) f(y,[deltain(t,D_strt,D_end), Tin(t,Fxf_strt,Fxf_end)]),...
    tspan, y0);

potting_simresults_forceXY(t,y,10,params);

%% --- simulation number 2 ---
% No steering angle sinusoidal wheel torque
Fxfmax = 1e3; % maximum torque
delta_max = 0; % maximum yawv rate

% ---- input functions ----
Hsigmoid = @(x) 0.5*(1+tanh(2*pi*6*(x)/(10)));
Ppulse = @(x,puls_strt,puls_end) Hsigmoid(x-puls_strt) + Hsigmoid(puls_end-x) - 1;

Tin = @(x,puls_strt,puls_end) Fxfmax*Ppulse(x,puls_strt,puls_end).*sin(pi*(x)/10 + pi/2);
deltain = @(x,puls_strt,puls_end) cos(2*pi*(0)/10).*Ppulse(x,puls_strt,puls_end)*delta_max;

D_strt = 20;
D_end = 30;

Fxf_strt = 15;
Fxf_end = 35;

%%%% ---- input test plots
% x = linspace(0,50,500);
% figure(2); clf;
% subplot(211);
% plot(x,Tin(x,T_strt,T_end),'LineWidth',2); box off
% subplot(212);
% plot(x,deltain(x,D_strt,D_end),'LineWidth',2); box off
% %%
tspan = [0 50];
y0 = zeros(5,1);
y0(3) = 0.1; y0(4) = 0;
[t,y] = ode45(@(t,y) f(y,[deltain(t,D_strt,D_end), Tin(t,Fxf_strt,Fxf_end)]),...
    tspan, y0);

potting_simresults_forceX(t,y,2,params,[D_strt D_end delta_max Fxf_strt Fxf_end Fxfmax]);

%% --- simulation number 3 ---
% 10 degree constnat steering angle with step wheel torque
Fxfmax = 500; % maximum torque
delta_max = deg2rad(0.5); % maximum steering angle

% ---- input functions ----
Hsigmoid = @(x) 0.5*(1+tanh(2*pi*6*(x)/(10)));
Ppulse = @(x,puls_strt,puls_end) Hsigmoid(x-puls_strt) + Hsigmoid(puls_end-x) - 1;

Tin = @(x,puls_strt,puls_end) Fxfmax*Ppulse(x,puls_strt,puls_end);
deltain = @(x,puls_strt,puls_end) sin(2*pi*x/10).*Ppulse(x,puls_strt,puls_end)*delta_max;

D_strt = 20;
D_end = 25;

Fxf_strt = 10;
Fxf_end = 25;

% %%% ---- input test plots
% % x = linspace(0,50,500);
% % figure(2); clf;
% % subplot(211);
% % plot(x,Tin(x,T_strt,T_end),'LineWidth',2); box off
% % subplot(212);
% % plot(x,deltain(x,D_strt,D_end),'LineWidth',2); box off
% %%
tspan = [0 50];
y0 = zeros(5,1);
y0(3) = 0.1; y0(4) = 0;
[t,y] = ode15s(@(t,y) f(y,[deltain(t,D_strt,D_end), Tin(t,Fxf_strt,Fxf_end)]),...
    tspan, y0);

potting_simresults_forceX(t,y,3,params,[D_strt D_end delta_max Fxf_strt Fxf_end Fxfmax]);

%% 
figure(4); clf;

plot(Tin(t,Fxf_strt,Fxf_end),Fyf(alphaf(vyf(deltain(t,D_strt,D_end),y(:,3),y(:,4),y(:,5)),vxf(deltain(t,D_strt,D_end),y(:,3),y(:,4),y(:,5))),Tin(t,Fxf_strt,Fxf_end)))


%% plots
distFig;

% figure(3); clf;
% tiledlayout('flow');
% 
% nexttile;
% plot(y(:,1),y(:,2)); grid on;
% ylabel('y'); xlabel('y'); 
% 
% nexttile;
% plot(t,y(:,3)); grid on;
% ylabel('v_x'); xlabel('t'); 
% 
% nexttile;
% plot(t,y(:,4)); grid on;
% ylabel('v_y'); xlabel('t'); 
% 
% nexttile;
% plot(t,y(:,5)); grid on;
% ylabel('yaw rate'); xlabel('t'); 
% 
% nexttile;
% plot(t,y(:,6)); grid on;
% ylabel('omega f'); xlabel('t'); 
% 
% nexttile;
% plot(t,y(:,7)); grid on;
% ylabel('omega r'); xlabel('t'); 
% 
% nexttile;
% plot(t,Tin(t,T_strt,T_end)); grid on;
% ylabel('T f'); xlabel('t'); 
% 
% nexttile;
% plot(t,deltain(t,D_strt,D_end)); grid on;
% ylabel('delta'); xlabel('t'); 