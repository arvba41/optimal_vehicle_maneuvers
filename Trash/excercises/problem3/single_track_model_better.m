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

% ---- intermediate equations ----
F_X = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,1);
F_Y = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,2);
M_Z = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,3);

Fx_f = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,4);
Fx_r = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,6);

Fy_f = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,5);
Fy_r = @(vx,vy,r,omegaf,omegar,delta) combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,7);
% ---- states ----
% x(1) -> x
% x(2) -> y
% x(3) -> vx
% x(4) -> vy
% x(5) -> r (or \dot psi)
% x(6) -> oemgaf
% x(7) -> oemgar
% ---- inputs ----
% u(1) --> delta 
% u(2) --> Tf
% ---- ODEs ----
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

%% Simulating the model
% --- simulation number 1 ---
% No steering angle only positive wheel torque
Tmax = 2000; % maximum torque
delta_max = 0; % maximum yawv rate

% ---- input functions ----
Hsigmoid = @(x) 0.5*(1+tanh(2*pi*6*(x)/(10)));
Ppulse = @(x,puls_strt,puls_end) Hsigmoid(x-puls_strt) + Hsigmoid(puls_end-x) - 1;

Tin = @(x,puls_strt,puls_end) Tmax*Ppulse(x,puls_strt,puls_end);
deltain = @(x,puls_strt,puls_end) cos(2*pi*(0)/10).*Ppulse(x,puls_strt,puls_end)*delta_max;

D_strt = 20;
D_end = 30;

T_strt = 10;
T_end = 40;

% ---- input test plots
% x = linspace(0,50,500);
% figure(2); clf;
% subplot(211);
% plot(x,Tin(x,T_strt,T_end),'LineWidth',2); box off
% subplot(212);
% plot(x,deltain(x,D_strt,D_end),'LineWidth',2); box off
% %%
tspan = [0 50];
y0 = zeros(7,1);
y0(3) = 0.1; y0(4) = 0;
[t,y] = ode45(@(t,y) f(y,[deltain(t,D_strt,D_end), Tin(t,T_strt,T_end)]),...
    tspan, y0);

potting_simresults(t,y,1,params,[D_strt D_end delta_max T_strt T_end Tmax]);

%% --- simulation number 2 ---
% No steering angle only positive wheel torque
Tmax = 0; % maximum torque
delta_max = deg2rad(5); % maximum yawv rate

% ---- input functions ----
Hsigmoid = @(x) 0.5*(1+tanh(2*pi*6*(x)/(10)));
Ppulse = @(x,puls_strt,puls_end) Hsigmoid(x-puls_strt) + Hsigmoid(puls_end-x) - 1;

Tin = @(x,puls_strt,puls_end) Tmax*Ppulse(x,puls_strt,puls_end);
deltain = @(x,puls_strt,puls_end) cos(2*pi*(x)/10).*Ppulse(x,puls_strt,puls_end)*delta_max;

D_strt = 25;
D_end = 30;

T_strt = 10;
T_end = 30;

% ---- input test plots
% x = linspace(0,50,500);
% figure(2); clf;
% subplot(211);
% subplot(212);
% plot(x,deltain(x,D_strt,D_end),'LineWidth',2); box off
% %%
tspan = [0 50];
y0 = zeros(7,1);
y0(3) = 50/3.6; y0(4) = 0;
[t,y] = ode45(@(t,y) f(y,[deltain(t,D_strt,D_end), Tin(t,T_strt,T_end)]),...
    tspan, y0);

potting_simresults(t,y,1,params,[D_strt D_end delta_max T_strt T_end Tmax]);

%% 
figure(2); clf;

deltat = @(t) deltain(t,D_strt,D_end);
for ii = 1:length(t) 
    [~,misc(ii)] = Fy_r(y(ii,3),y(ii,4),y(ii,5),y(ii,6),y(ii,7),deltat(t(ii)));
    Fxf(ii) = misc(ii).Fxf;
    Fxr(ii) = misc(ii).Fxr;
    Fyf(ii) = misc(ii).Fyf;
    Fyr(ii) = misc(ii).Fyr;
    alphaf(ii) = misc(ii).alphaf;
    alphar(ii) = misc(ii).alphar;
    kappaf(ii) = misc(ii).kappaf;
    kappar(ii) = misc(ii).kappar;
end

Fzf = params.m*params.g*params.lr/(params.lf+params.lr);
Fzr = params.m*params.g*params.lf/(params.lf+params.lr);

%%
clf; tiledlayout(4,2)
ax(1) = nexttile;
plot(t,sqrt(Fxf.^2 + Fyf.^2)/Fzf,'LineWidth',2); 
ylabel('$\sqrt{F_{x,f}^2 + F_{y,f}^2}/F_{z,f}$ [-]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex'); 

ax(2) = nexttile;
plot(t,sqrt(Fxr.^2 + Fyr.^2)/Fzr,'LineWidth',2); 
ylabel('$\sqrt{F_{x,r}^2 + F_{y,r}^2}/F_{z,r}$ [-]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex'); 

ax(3) = nexttile;
plot(t,deg2rad([alphaf; alphar]),'LineWidth',2); 
ylabel('$\alpha$ [deg]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex'); 
legend('f','r','Interpreter','latex');

ax(4) = nexttile;
plot(t,[kappaf; kappar],'LineWidth',2); 
ylabel('$\kappa$ [-]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex'); 
legend('f','r','Interpreter','latex');


af = linspace(-pi/6,pi/6,51); ar = af;
kf = linspace(-1,1,51); kr = kf;

Fzf = params.m*params.g*params.lr/(params.lf+params.lr);
Fzr = params.m*params.g*params.lf/(params.lf+params.lr);

for jj = 1:length(kf)
    for ii = 1:length(af)
        [~,Fxfm(ii,jj),Fyfm(ii,jj),Fxrm(ii,jj),Fyrm(ii,jj)] = magic_formula_friction_elipses(params,Fzf,Fzr,af(ii),ar(ii),kf(jj),kr(jj),100);
    end
end

nexttile([2 1]);
for jj = 1:length(kf)
    for ii = 1:length(af)
        val1(ii,jj) = Fxfm(ii,jj);
        val2(ii,jj) = Fyfm(ii,jj);
    end
end
mesh(kf,rad2deg(af),sqrt(val1.^2 + val2.^2)./Fzf); hold on;
plot3(kappaf,rad2deg(alphaf),sqrt(Fxf.^2+Fyf.^2)./Fzf,'LineWidth',2); hold off;
ylabel('$\alpha_f$ [deg]','Interpreter','latex'); 
xlabel('$\kappa_f$ [-]','Interpreter','latex'); 
zlabel('$\sqrt{F_{x,r}^2 + F_{y,r}^2}/F_{z,r}$ [-]','Interpreter','latex'); 
title('Resulting tire force (FE) (front)','Interpreter','latex')
xlim([-1,1]); ylim(rad2deg([-pi/6,pi/6]))

nexttile([2 1]);
for jj = 1:length(kr)
    for ii = 1:length(ar)
        val1(ii,jj) = Fxrm(ii,jj);
        val2(ii,jj) = Fyrm(ii,jj);
    end
end
mesh(kr,rad2deg(ar),sqrt(val1.^2 + val2.^2)./Fzr); hold on;
plot3(kappar,rad2deg(alphar),sqrt(Fxr.^2+Fyr.^2)./Fzr,'LineWidth',2); hold off;
ylabel('$\alpha_f$ [deg]','Interpreter','latex'); 
xlabel('$\kappa_f$ [-]','Interpreter','latex'); 
zlabel('$\sqrt{F_{x,f}^2 + F_{y,f}^2}/F_{z,f}$ [-]','Interpreter','latex'); 
title('Resulting tire force (FE) (rear)','Interpreter','latex')
xlim([-1,1]); ylim(rad2deg([-pi/6,pi/6]))

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%% Trash
% 
% %% --- simulation number 2 ---
% % No steering angle sinusoidal wheel torque
% Tmax = 400; % maximum torque
% delta_max = 0; % maximum yawv rate
% 
% % ---- input functions ----
% Hsigmoid = @(x) 0.5*(1+tanh(2*pi*6*(x)/(10)));
% Ppulse = @(x,puls_strt,puls_end) Hsigmoid(x-puls_strt) + Hsigmoid(puls_end-x) - 1;
% 
% Tin = @(x,puls_strt,puls_end) Tmax*Ppulse(x,puls_strt,puls_end).*sin(pi*(x)/10 + pi/2);
% deltain = @(x,puls_strt,puls_end) cos(2*pi*(0)/10).*Ppulse(x,puls_strt,puls_end)*delta_max;
% 
% D_strt = 20;
% D_end = 30;
% 
% T_strt = 15;
% T_end = 35;
% 
% %%%% ---- input test plots
% % x = linspace(0,50,500);
% % figure(2); clf;
% % subplot(211);
% % plot(x,Tin(x,T_strt,T_end),'LineWidth',2); box off
% % subplot(212);
% % plot(x,deltain(x,D_strt,D_end),'LineWidth',2); box off
% % %%
% tspan = [0 50];
% y0 = zeros(7,1);
% y0(3) = 0.1; y0(4) = 0;
% [t,y] = ode45(@(t,y) f(y,[deltain(t,D_strt,D_end), Tin(t,T_strt,T_end)]),...
%     tspan, y0);
% 
% potting_simresults(t,y,2,params,[D_strt D_end delta_max T_strt T_end Tmax]);
% 
% %% --- simulation number 3 ---
% % 10 degree constnat steering angle with step wheel torque
% Tmax = 500; % maximum torque
% delta_max = deg2rad(3); % maximum steering angle
% 
% % ---- input functions ----
% Hsigmoid = @(x) 0.5*(1+tanh(2*pi*6*(x)/(10)));
% Ppulse = @(x,puls_strt,puls_end) Hsigmoid(x-puls_strt) + Hsigmoid(puls_end-x) - 1;
% 
% Tin = @(x,puls_strt,puls_end) Tmax*Ppulse(x,puls_strt,puls_end);
% deltain = @(x,puls_strt,puls_end) sin(2*pi*x/10).*Ppulse(x,puls_strt,puls_end)*delta_max;
% 
% D_strt = 30;
% D_end = 40;
% 
% T_strt = 5;
% T_end = 25;
% 
% % %%% ---- input test plots
% % % x = linspace(0,50,500);
% % % figure(2); clf;
% % % subplot(211);
% % % plot(x,Tin(x,T_strt,T_end),'LineWidth',2); box off
% % % subplot(212);
% % % plot(x,deltain(x,D_strt,D_end),'LineWidth',2); box off
% % %%
% tspan = [0 50];
% y0 = zeros(7,1);
% y0(3) = 0.1; y0(4) = 0;
% [t,y] = ode45(@(t,y) f(y,[deltain(t,D_strt,D_end), Tin(t,T_strt,T_end)]),...
%     tspan, y0);
% 
% potting_simresults(t,y,3,params,[D_strt D_end delta_max T_strt T_end Tmax]);
% 
% %% plots
% distFig;
% 
% % figure(3); clf;
% % tiledlayout('flow');
% % 
% % nexttile;
% % plot(y(:,1),y(:,2)); grid on;
% % ylabel('y'); xlabel('x'); 
% % 
% % nexttile;
% % plot(t,y(:,3)); grid on;
% % ylabel('v_x'); xlabel('t'); 
% % 
% % nexttile;
% % plot(t,y(:,4)); grid on;
% % ylabel('v_y'); xlabel('t'); 
% % 
% % nexttile;
% % plot(t,y(:,5)); grid on;
% % ylabel('yaw rate'); xlabel('t'); 
% % 
% % nexttile;
% % plot(t,y(:,6)); grid on;
% % ylabel('omega f'); xlabel('t'); 
% % 
% % nexttile;
% % plot(t,y(:,7)); grid on;
% % ylabel('omega r'); xlabel('t'); 
% % 
% % nexttile;
% % plot(t,Tin(t,T_strt,T_end)); grid on;
% % ylabel('T f'); xlabel('t'); 
% % 
% % nexttile;
% % plot(t,deltain(t,D_strt,D_end)); grid on;
% % ylabel('delta'); xlabel('t'); 