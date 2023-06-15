% clear all; clc;
% 
% Xfin = 50; % finishing position 
% mass = 1000; % mass of the car
% mu = 0.5; % friction
% g = 9.8; % acceleration due to gravity

%% Car Avoiding an obstacle
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

N = 40; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
xpos = X(1,:);
ypos = X(2,:);
xspeed = X(3,:);
yspeed = X(4,:);
U = opti.variable(2,N);   % control trajectory (throttle)
T = opti.variable();      % final time

% ---- objective          ---------
opti.minimize(T);         % minimize time

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
Flim = U(1,:).^2 +  U(2,:).^2; % Force limit equation
opti.subject_to(0<=Flim<=(mass*mu*g)^2); % Force is limited

% ---- path constraints -----------
% y1curve = @(x) sin(2*pi*x/(2*Xfin))*2 + 5; % bottom road limits
yucurve = @(x) sin(2*pi*x/(2*Xfin))*2 + 7; % upper road limits

% opti.subject_to(ypos>y1curve(xpos)); % be above the lower limit
opti.subject_to(ypos<yucurve(xpos)); % be below the upper limit

% opti.subject_to(xpos>0); % be above zero
opti.subject_to(xspeed>0); % be above zero
opti.subject_to(ypos>0); % be above zero

% opti.subject_to(xpos(N+1)>xpos(N)); % position on x should always increase
% opti.subject_to(yspeed>0); % be above zero

% ---- boundary conditions --------
% define all the initial values here 
opti.subject_to(xpos(1)==0);   % start at position x ...
opti.subject_to(ypos(1)==6);   % start at position y ... 
opti.subject_to(xpos(N+1)==2*Xfin);  % finish line at position x 
opti.subject_to(ypos(N+1)==ypos(1));  % finish line at position y 
opti.subject_to(xspeed(1)==0);   % start with speed x ...
opti.subject_to(yspeed(1)==0);   % start with speed y ...

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
% define all the initial values here 
opti.set_initial(xpos, 0);
% opti.set_initial(ypos, 6);
opti.set_initial(xspeed, 1);
opti.set_initial(yspeed, 1);
opti.set_initial(T, 1);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

%% converting to readable values
prob3.x = sol.value(xpos); prob3.y = sol.value(ypos);
prob3.vx = sol.value(xspeed); prob3.vy = sol.value(yspeed);
prob3.ux = sol.value(U(1,:)); prob3.uy = sol.value(U(2,:));
prob3.topt = sol.value(T); prob3.tvec = linspace(0,sol.value(T),N+1);

prob3.cons.Flim = mass*mu*g; 
% prob3.cons.yllim = y1curve(prob3.x); 
prob3.cons.yulim = yucurve(prob3.x);

%% plotting
%%%% debugging
% prob3.x = opti.debug.value(xpos); prob3.y = opti.debug.value(ypos);
% prob3.vx = opti.debug.value(xspeed); prob3.vy = opti.debug.value(yspeed);
% prob3.ux = opti.debug.value(U(1,:)); prob3.uy = opti.debug.value(U(2,:));
% prob3.topt = opti.debug.value(T); prob3.tvec = linspace(0,opti.debug.value(T),N+1);
% 
% prob3.cons.Flim = mass*mu*g; 
% % prob3.cons.yllim = y1curve(prob3.x); 
% prob3.cons.yulim = yucurve(prob3.x);

f = figure(3); clf;
optim_plot_prob3(prob3);
