clear all;
clc; 

load('opti_mov')

%% parameters and initial conditions
% Given values
v = 20; % m/s
% A = 12; % m

% initial positions
Ypos = 0; 
Xpos = 0; 

% parameters
params.m = 2000;
params.g = 9.81;
params.mu = 0.6; % wet asphalt 

% obstacle
obst.c = 30; % center
obst.w = 2.5; % width 
obst.h = 3; % height
obst.n = 12; % sharpness 

%% NLP 
N = 50;                 % number of control intervals

opti = casadi.Opti();   % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
xpos    = X(1,:);
ypos    = X(2,:);
vx      = X(3,:);
vy      = X(4,:);
U = opti.variable(2,N);   % control trajectory (steering rate and front wheel force)
Fx      = U(1,:);
Fy      = U(2,:);
T = opti.variable();      % final time

% ---- objective          ---------
J = T; 
opti.minimize(J);         % minimize time

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

% ---- path constrains
opti.subject_to(obstacle(xpos,ypos,obst) >= 1);
% opti.subject_to(path(xpos,ypos,R1o,R2o) <= 1);

% ---- boundary conditions --------
opti.subject_to(xpos(1)==0);        
opti.subject_to(ypos(1)==0);  
opti.subject_to(xpos(N+1)==2*obst.c);        
opti.subject_to(ypos(N+1)==obst.h);  
opti.subject_to(vx(1)==0);   
opti.subject_to(vy(1)==0);   

% ---- force constrains ----
umax = params.mu*params.g*params.m;
opti.subject_to((Fx.^2 + Fy.^2)<=umax^2);   

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- intial guess ----
% opti.set_initial(xpos, 0);
% opti.set_initial(ypos, 0);
% opti.set_initial(vx, 0);
% opti.set_initial(vy, prob1.vy);
opti.set_initial(Fx, 0.1);
opti.set_initial(Fy, 0);
opti.set_initial(T, 10);

% opti.set_initial(xpos, prob1.xpos);
% opti.set_initial(ypos, prob1.ypos);
% opti.set_initial(vx, prob1.vx);
% opti.set_initial(vy, prob1.vy);
% opti.set_initial(Fx, prob1.Fx);
% opti.set_initial(Fy, prob1.Fy);
% opti.set_initial(T, prob1.topt);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
%opti.solver('osqp'); % set numerical backend
sol = opti.solve();   % actual solve

%% data recording 
prob1.xpos = opti.debug.value(xpos);
prob1.ypos = opti.debug.value(ypos);
prob1.vx = opti.debug.value(vx);
prob1.vy = opti.debug.value(vy);
prob1.Fx = opti.debug.value(Fx);
prob1.Fy = opti.debug.value(Fy);
prob1.topt = opti.debug.value(T); 
prob1.t = linspace(0,opti.debug.value(T),N+1);

%% plottings

figure(1); clf;
tiledlayout('flow');

nexttile;
plot(prob1.xpos,prob1.ypos,'x-'); hold on;
xlabel('$x$ [m]','Interpreter','latex');
ylabel('$y$ [m]','Interpreter','latex');
fimplicit(@(x,y) obstacle(x,y,obst) - 1,[0 2*obst.c 1.1*obst.h],'--','Color','k'); hold on;
% fimplicit(@(x,y) path(x,y,R1i,R2i) - 1,[-R1o R1o -R2o R2o],'--','Color','k'); hold on;
% fimplicit(@(x,y) path(x,y,R1o,R2o) - 1,[-R1o R1o -R2o R2o],'--','Color','k'); hold off;

nexttile;
plot(prob1.t,prob1.vx*3.6);
ylabel('$v_x$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.t,prob1.vy*3.6);
ylabel('$v_y$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.t(2:end),prob1.Fx,'DisplayName','$u_1$'); hold on
plot(prob1.t(2:end),prob1.Fy,'DisplayName','$u_2$'); hold off
yline(-umax,'--','LineWidth',2);
yline(umax,'--','LineWidth',2);
ylabel('$u$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%% analytical solution

%%%% determining theta
% simulation 
theta_sim = pi/2 - atan((prob1.ypos(end-6) - prob1.ypos(end-8))/(prob1.xpos(end-6) - prob1.xpos(end-8)));

% analytical
B = prob1.ypos(end - 7);
A = prob1.xpos(end - 7);

gamma = atan(B/A);

theta_anl = (gamma + asin(3*sin(gamma)))/2;

tvec = linspace(0,prob1.t(end),100);

mu = params.mu;
g = params.g;
m = params.m;

xanl = -(mu*g*sin(theta_anl))*tvec.^2/2 + v*tvec;
yanl = (mu*g*cos(theta_anl))*tvec.^2/2;

plot(axx(1),xanl,yanl);

u1 = -m*mu*g*sin(theta_anl);
u2 = m*mu*g*cos(theta_anl);

plot(axx(1),xanl,yanl);

yline(ax(3),u1,'b','--');
yline(ax(3),u2,'r','--');
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