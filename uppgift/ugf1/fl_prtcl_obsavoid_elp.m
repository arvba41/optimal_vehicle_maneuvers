function [oplsol,elps] = fl_prtcl_obsavoid_elp(mdl,obst,cons,optops,val)
% Friction-limited Particle with Obstacle Avoidance using CasADi:
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

Xfin = mdl.Xfin;    % finishing position x
mass = mdl.mass;    % mass of the car
mu = mdl.mu;        % friction
g = mdl.g;          % acceleration due to gravity

% obstacle parameters
Xa = obst.Xa;       % center of the elipse
R1 = obst.R1;       % radius of elipse in x direction
R2 = obst.R2;       % radius of elipse in x direction
pow = obst.pow;     % curvature of elipse 

Ymax = cons.Ymax;   % upper limit for the y direction
vxinit = cons.vxinit; % initial speed x direction

N = optops.N;        % number of control intervals for the optimization

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
elps = @(x,y) -((x-Xa)/R1).^pow -(y/R2).^pow + 1; % obstacle 

opti.subject_to(elps(xpos,ypos)<=0); % ensure that we are far from the elipse

opti.subject_to(xspeed>0); % x speed should not be negative to avoid negative x position
opti.subject_to(0<=ypos<Ymax); % position in y to not go below zero and ensuring that the y position does not go to inf

% ---- boundary conditions --------
% define all the initial values here 
opti.subject_to(xpos(1)==0);   % start at position x ...
opti.subject_to(ypos(1)==1);   % start at position y ... 
opti.subject_to(xspeed(1)==vxinit);   % start with speed x ...
opti.subject_to(yspeed(1)==0);   % start with speed y ...

opti.subject_to(xpos(N+1)==2*Xfin);  % finish line at position x 
opti.subject_to(ypos(N+1)==ypos(1));  % finish line at position y 

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
% define all the initial values here
if val ~= []
    opti.set_initial(xpos, val.x);
    opti.set_initial(ypos, val.y);
    opti.set_initial(xspeed, val.vx);
    opti.set_initial(yspeed, val.vy);
    opti.set_initial(T, val.topt);
    opti.set_initial(Fx, val.ux);
    opti.set_initial(Fy, val.uy);
else
    opti.set_initial(xpos, 0);
    opti.set_initial(ypos, 1);
    opti.set_initial(xspeed, vxinit);
    opti.set_initial(yspeed, 0);
    opti.set_initial(T, 1);
    opti.set_initial(U(1,:), 1000);
    opti.set_initial(U(2,:), 0);
end
% ---- solve NLP ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

% ---- results sotrage ----
oplsol.x = sol.value(xpos); oplsol.y = sol.value(ypos);
oplsol.vx = sol.value(xspeed); oplsol.vy = sol.value(yspeed);
oplsol.ux = sol.value(U(1,:)); oplsol.uy = sol.value(U(2,:));
oplsol.topt = sol.value(T); oplsol.tvec = linspace(0,sol.value(T),N+1);

oplsol.cons.Flim = mass*mu*g;

end