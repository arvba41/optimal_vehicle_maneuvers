%% Maximum entry speed
clc
clear all

mass = 1400;
mass = 700;
g = 9.82;
my = 0.15; % torr asfalt

x_a = 25;
x_a = 50;
n = 6;
R1 = 4.5; 
R2 = 2.5;

N = 40; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
pos_x   = X(1,:);
pos_y   = X(2,:);
speed_x = X(3,:);
speed_y = X(4,:);
U = opti.variable(2,N);   % control trajectory (throttle)
T = opti.variable(); 

opti.minimize(-speed_x(1)); % race in maximum start speed
% opti.minimize(T); % Minimum time

% ---- dynamic constraints --------
f = @(x,u) [x(3);x(4);u(1)/mass;u(2)/mass]; % dx/dt = f(x,u)

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

% ---- path constraints -----------
opti.subject_to(-((pos_x-x_a)/R1).^n - (pos_y/R2).^n  + 1 <= 0);           % Obstacle
opti.subject_to(U(1,:).^2 + U(2,:).^2 == (mass*my*g)^2);           % control is limited

% opti.subject_to((U(1,:).^2 + U(2,:).^2) <= (mass*my*g)^2);           % control is limited

% ---- boundary conditions --------
opti.subject_to(pos_x(1)==0);   % start at position 0 ...
opti.subject_to(pos_x(N+1)==2*x_a); % finish line at position 1
opti.subject_to(pos_y(1)==1);   % start at position 1 ...
opti.subject_to(pos_y(N+1)==1); % finish line at position 1
opti.subject_to(speed_y(1)==0);

% opti.subject_to(speed_x(1)==70/3.6);
% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive
opti.subject_to(speed_x >= 0); % finish line at position 1
opti.subject_to(0 <= pos_y <= 5);

% ---- initial values for solver ---
opti.set_initial(speed_x, 10);
% opti.set_initial(speed_y, 0);
opti.set_initial(T, 5);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

%% ---- post-processing debugging       ------
% y = R2*(((x_vec-x_a)/R1).^n-1).^(1/n);
f = @(x,y) ((x-x_a)/R1).^n + (y/R2).^n - 1;
figure(1); clf;
hold on
plot(opti.debug.value(pos_x),opti.debug.value(pos_y),'x-');
fimplicit(f,[0 2*x_a 0 5])
legend('pos','obstacle','Location','northwest')

%% solution found
f = @(x,y) ((x-x_a)/R1).^n + (y/R2).^n - 1;
figure(10) 
clf
hold on
plot(sol.value(pos_x),sol.value(pos_y),'x-');
fimplicit(f,[0 2*x_a 0 5])
hold off
grid on
legend('pos','obstacle','Location','northwest')

figure(20)
clf
subplot(2,1,1)
plot(sol.value(pos_x),sol.value(speed_x)*3.6);
title('Speed X-axis')

subplot(2,1,2)
plot(sol.value(pos_x),sol.value(speed_y*3.6));
title('Speed Y-axis')

end_time = sol.value(T)
max_speed = sol.value(speed_x(1))