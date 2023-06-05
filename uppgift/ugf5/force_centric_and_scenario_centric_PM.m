clear all; clc
%% parameters

Xfin = 20;          % finishing position x
mass = 2000;        % mass of the car
g = 9.8;            % acceleration due to gravity

% obstacle parameters
Xa = Xfin+5;            % center of the elipse
R1 = 5;             % radius of elipse in x direction
R2 = 3;             % radius of elipse in x direction
pow = 20;            % curvature of elipse 

Ymax = 4;           % upper limit for the y direction
vxinit = 20;        % initial speed x direction

N = 60;            % number of control intervals for the optimization

%% OCP -- particle model
opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
xpos = X(1,:);
ypos = X(2,:);
xspeed = X(3,:);
yspeed = X(4,:);
U = opti.variable(2,N);   % control trajectory (throttle)
Fx = U(1,:);
Fy = U(2,:);
T = opti.variable();      % final time
mu = opti.variable();      % friction

% ---- objective          ---------
opti.minimize(mu);         % minimize friction

% ---- dynamic constraints --------
% x(1) --> x
% x(2) --> y
% x(3) --> vx
% x(4) --> vy
f = @(x,u) [x(3);... x' = vx
            x(4);... y' = vy
            u(1)/mass;... vx' = u1/m
            u(2)/mass];% vy' = u2/m
% dx/dt = f(x,u)

dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- force constraints -----------
opti.subject_to(Fx <= 0)
Flim = Fx.^2 +  Fy.^2; % Force limit equation
Fmax = mass*mu*g;
opti.subject_to(Flim <= Fmax^2); % Force is limited

% ---- path constraints -----------
elps = @(x,y) -((x-Xa)/R1).^pow -(y/R2).^pow + 1; % obstacle 

opti.subject_to(elps(xpos,ypos)<=0); % ensure that we are far from the elipse

opti.subject_to(xspeed>0); % x speed should not be negative to avoid negative x position
opti.subject_to(0<=ypos<Ymax); % position in y to not go below zero and ensuring that the y position does not go to inf

% ---- boundary conditions --------
% define all the initial values here 
opti.subject_to(xpos(1)==0);   % start at position x ...
opti.subject_to(ypos(1)==0);   % start at position y ... 
opti.subject_to(xspeed(1)==vxinit);   % start with speed x ...
opti.subject_to(yspeed(1)==0);   % start with speed y ...

opti.subject_to(xpos(N+1)==25);  % finish line at position x 

% ---- misc. constraints  ----------
opti.subject_to(T >= 0); % Time must be positive
opti.subject_to(0 <= mu <= 1); % mu must be positive

opti.set_initial(xpos, 0);
opti.set_initial(ypos, 0);
opti.set_initial(xspeed, vxinit);
opti.set_initial(yspeed, 0);
opti.set_initial(T, 1.6);
opti.set_initial(mu, 0.1);
opti.set_initial(U(1,:), 1000);
opti.set_initial(U(2,:), 1000);

% ---- solve NLP ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

%% ---- results sotrage ----
oplsol.x = opti.debug.value(xpos); oplsol.y = opti.debug.value(ypos);
oplsol.vx = opti.debug.value(xspeed); oplsol.vy = opti.debug.value(yspeed);
oplsol.ux = opti.debug.value(U(1,:)); oplsol.uy = opti.debug.value(U(2,:));
oplsol.topt = opti.debug.value(T); oplsol.tvec = linspace(0,opti.debug.value(T),N+1);
oplsol.Fmax = mass*opti.debug.value(mu)*g; oplsol.mu = opti.debug.value(mu);

oplsol.cons.Flim = mass*mu*g;

%% plots
figure(10); clf; 

idx = find(round(elps(oplsol.x,oplsol.y),1) == 0);
idx(1) = [];

subplot(3,2,1:2); 
plot(oplsol.x,oplsol.y); box off; hold on;
ylabel('y [m]'), xlabel('x [m]');
fimplicit(@(x,y) elps(x,y),[0 25 0 4],'--','Color','k');
xlim([0 oplsol.x(idx)]);

subplot(323); 
yyaxis left
plot(oplsol.tvec,oplsol.vx*3.6); box off;
ylabel('v_x [km/h]'); 
yyaxis right
plot(oplsol.tvec,oplsol.vy*3.6); box off;
ylabel('v_y [km/h]'); xlabel('t [s]');
xlim([0 oplsol.tvec(idx)]);

subplot(324); 
plot(oplsol.tvec,sqrt(oplsol.vx.^2 + oplsol.vy.^2)*3.6); box off;
ylabel('v [km/h]'); xlabel('t [s]');
xlim([0 oplsol.tvec(idx)]);

subplot(325); 
yyaxis left
plot(oplsol.tvec(2:end),oplsol.ux/1e3); box off;
ylabel('F_x [kN]'); 
yyaxis right
plot(oplsol.tvec(2:end),oplsol.uy/1e3); box off;
ylabel('F_y [kN]'); xlabel('t [s]');
xlim([0 oplsol.tvec(idx)]);

subplot(326); 
plot(oplsol.tvec(2:end),sqrt(oplsol.ux.^2 + oplsol.uy.^2)/1e3); box off;
ylabel('F [kN]'); xlabel('t [s]');
yline(oplsol.Fmax/1e3,'--')
xlim([0 oplsol.tvec(idx)]);

%% Determining theta 

idx = find(round(elps(oplsol.x,oplsol.y),1) == 0);

theta_sim  = atan(diff(oplsol.y(idx))/diff(oplsol.x(idx)));

% scenario centric 
oplsol.Fcx = oplsol.ux.*cos(theta_sim) + oplsol.uy.*sin(theta_sim);
oplsol.Fcy = -oplsol.ux.*sin(theta_sim) + oplsol.uy.*cos(theta_sim);

% vehicle centric 
oplsol.psi_sim = atan(diff(oplsol.y)./diff(oplsol.x));
oplsol.Fvx = oplsol.ux.*cos(oplsol.psi_sim) + oplsol.uy.*sin(oplsol.psi_sim);
oplsol.Fvy = -oplsol.ux.*sin(oplsol.psi_sim) + oplsol.uy.*cos(oplsol.psi_sim);

figure(20); clf;

textwidth = 15;
textheight = 10;
figsize = [textwidth, textheight];

subplot(321);
plot(oplsol.tvec(2:end),oplsol.Fcy/1e3); box off;
ylabel('$F_{c,y}$ [kN]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
yline(oplsol.Fmax/1e3,'--')
xlim([0 oplsol.tvec(idx(1))]);

subplot(322);
plot(oplsol.tvec(2:end),oplsol.Fcx/1e3); box off;
ylabel('$F_{c,x}$ [kN]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
xlim([0 oplsol.tvec(idx(1))]);

subplot(323);
plot(oplsol.tvec(2:end),oplsol.Fvy/1e3); box off;
ylabel('$F_{v,y}$ [kN]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
yline(oplsol.Fmax/1e3,'--')
xlim([0 oplsol.tvec(idx(1))]);

subplot(324);
plot(oplsol.tvec(2:end),oplsol.Fvx/1e3); box off;
ylabel('$F_{v,x}$ [kN]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
xlim([0 oplsol.tvec(idx(1))]);

subplot(325);
plot(oplsol.tvec(2:end),oplsol.uy/1e3); box off;
ylabel('$F_{y}$ [kN]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
yline(oplsol.Fmax/1e3,'--')
xlim([0 oplsol.tvec(idx(1))]);

subplot(326);
plot(oplsol.tvec(2:end),oplsol.ux/1e3); box off;
ylabel('$F_{x}$ [kN]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
xlim([0 oplsol.tvec(idx(1))]);

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

% print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\prob5_opt_avoid_controlforce.pdf