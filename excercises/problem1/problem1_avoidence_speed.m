clear all; clc;

Xfin = 50; % finishing position 
mass = 1000; % mass of the car
mu = 0.5; % friction
g = 9.8; % acceleration due to gravity

% obstacle parameters
Xa = Xfin; R1 = 10; R2 = 2; Ylim = 1; pow = 2;

Ymax = 5; % upper limit 

%% Car Avoiding an obstacle
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

N = 100; % number of control intervals

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
vo = xspeed(T)^2 + yspeed(T)^2;  % vo at time T
opti.minimize(-vo);         % minimize the speed

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
% yucurve = @(x) sin(2*pi*x/(2*Xfin))*2 + 7; % upper road limits
obst = @(x,y) -((x-Xa)/R1).^pow -(y/R2).^pow + Ylim; % obstacle 

yobst = @(x) (Ylim -((x-Xa)/R1).^pow).^(1/pow)*R2; %% y value 

opti.subject_to(obst(xpos,ypos)<=0); % ensure that we are far from the elipse

opti.subject_to(xspeed>0); % x speed should not be negative to avoid negative x position
opti.subject_to(0<=ypos<Ymax); % position in y to not go below zero and ensuring that the y position does not go to inf

% opti.subject_to(ypos<Ymax); % ensuring that the y position does not go to inf

% ---- boundary conditions --------
% define all the initial values here 
opti.subject_to(xpos(1)==0);   % start at position x ...
opti.subject_to(ypos(1)==1);   % start at position y ... 
opti.subject_to(xspeed(1)==0);   % start with speed x ...
opti.subject_to(yspeed(1)==0);   % start with speed y ...

opti.subject_to(xpos(N+1)==2*Xfin);  % finish line at position x 
opti.subject_to(ypos(N+1)==ypos(1));  % finish line at position y 

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
% define all the initial values here 
opti.set_initial(xpos, 0);
opti.set_initial(ypos, 1);
opti.set_initial(xspeed, 0);
opti.set_initial(yspeed, 0);
opti.set_initial(T, 1);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

%% converting to readable values
prob6.x = sol.value(xpos); prob6.y = sol.value(ypos);
prob6.vx = sol.value(xspeed); prob6.vy = sol.value(yspeed);
prob6.ux = sol.value(U(1,:)); prob6.uy = sol.value(U(2,:));
prob6.topt = sol.value(T); prob6.tvec = linspace(0,sol.value(T),N+1);

prob6.cons.Flim = mass*mu*g; 
% prob5.cons.obst = obst(0:2*Xfin,0:Yulim); 

%% plotting
%%%% debugging
prob6.x = opti.debug.value(xpos); prob6.y = opti.debug.value(ypos);
prob6.vx = opti.debug.value(xspeed); prob6.vy = opti.debug.value(yspeed);
prob6.ux = opti.debug.value(U(1,:)); prob6.uy = opti.debug.value(U(2,:));
prob6.topt = opti.debug.value(T); prob6.tvec = linspace(0,opti.debug.value(T),N+1);

prob6.cons.Flim = mass*mu*g; 
prob6.cons.obst = obst(prob6.x,prob6.y);
prob6.cons.yobst = yobst(prob6.x);
% prob5.cons.obst = obst(linspace(0,2*Xfin,100),linspace(0,Yulim,100)); 

% prob5.cons.yllim = y1curve(prob5.x); 
% prob5.cons.yulim = yucurve(prob5.x);

f = figure(5); clf;
optim_plot_prob5(prob6)
