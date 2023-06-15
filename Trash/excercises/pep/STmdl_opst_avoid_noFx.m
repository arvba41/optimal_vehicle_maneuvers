clear all; clc;

%% ---- model parameterization ----
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

% start and final distance
Xsrt    = 0;
Xfin    = 80/2;
Xend    = 2*Xfin;
Ystrt   = 1;
Yfin    = 1;

% obstacle parameters
Xa = Xfin;  % center of the obsticle 
R1 = 5;     % width of the obsticle 
R2 = 2;     % height of the obsticle 
pow = 4;    % squareness of the obsticle 

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;
Ymax = 3; % upper limit 

Vxstart = 60/3.6;


%% Car Avoiding an obstacle
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

% ST model parameters
m   = params.m;
lf  = params.lf; 
lr  = params.lr;
g   = params.g;
Izz = params.Izz;
Iw  = params.Iw;
Re  = params.Re;

N = 60;                 % number of control intervals

opti = casadi.Opti();   % Optimization problem

% ---- decision variables ---------
X = opti.variable(7,N+1); % state trajectory
xpos    = X(1,:);
ypos    = X(2,:);
vx      = X(3,:);
vy      = X(4,:);
r       = X(5,:);
delta   = X(6,:);
psi     = X(7,:);
U = opti.variable(1,N);   % control trajectory (steering rate)
ddelta   = U(1,:);
T = opti.variable();      % final time

% ---- objective          ---------
opti.minimize(T);         % minimize time

% ---- dynamic constraints -----
% ---- parameters ----
muxf = params.mux(1); muxr = params.mux(2); 
muyf = params.muy(2); muyr = params.muy(2);
Cxf = params.Cx(1); Cxr = params.Cx(2);
Cyf = params.Cy(1); Cyr = params.Cy(2);
Bxf = params.Bx(1); Bxr = params.Bx(2);
Byf = params.By(1); Byr = params.By(2);
Exf = params.Ex(1); Exr = params.Ex(2);
Eyf = params.Ey(1); Eyr = params.Ey(2);
% ---- independent wheel velocities ----
vxf = @(vx,vy,r,delta) vx.*cos(delta) + sin(delta).*(vy + lf.*r); 
vyf = @(vx,vy,r,delta) cos(delta).*(vy + lf.*r) - vx.*sin(delta);
vxr = @(vx,vy,r,delta) vx;
vyr = @(vx,vy,r,delta) vy + lr.*r;
% ---- determining the lateral slips ----
alphaf = @(vx,vy,r,delta) -atan(vyf(vx,vy,r,delta)./vxf(vx,vy,r,delta));
alphar = @(vx,vy,r,delta) -atan(vyr(vx,vy,r,delta)./vxr(vx,vy,r,delta));
% ---- determining the lateral forces on the wheels from Fy ----
% maximum forces
Fzf = m*g*lr/(lf+lr); 
Fzr = m*g*lf/(lf+lr);
Dxf = muxf.*Fzf; Dxr = muxr.*Fzr;
Dyf = muyf.*Fzf; Dyr = muyr.*Fzr;
Fyf = @(vx,vy,r,delta) Dyf.*sin(Cyf.*atan(Byf.*alphaf(vx,vy,r,delta)-Eyf.*(Byf.*alphaf(vx,vy,r,delta) - atan(Byf.*alphaf(vx,vy,r,delta)))));
Fyr = @(vx,vy,r,delta) Dyr.*sin(Cyr.*atan(Byr.*alphar(vx,vy,r,delta)-Eyr.*(Byr.*alphar(vx,vy,r,delta) - atan(Byr.*alphar(vx,vy,r,delta)))));
% ---- the total forces and moments acting on the vehcile ----
F_X = @(vx,vy,r,delta) -sin(delta).*Fyf(vx,vy,r,delta);  
F_Y = @(vx,vy,r,delta) cos(delta).*Fyf(vx,vy,r,delta) + Fyr(vx,vy,r,delta);
M_Z = @(vx,vy,r,delta) lf*cos(delta).*Fyf(vx,vy,r,delta) - lr*Fyr(vx,vy,r,delta);

% ---- ODE ----
f = @(x,u) [ ... posistion dynamics ...
                    x(3)*cos(x(7)) - x(4)*sin(x(7)); ...    \dot x = vx*cos(psi) - vy*sin(psi) 
                    x(3)*sin(x(7)) + x(4)*cos(x(7)); ...    \dot y = vx*sin(psi) - vy*cos(psi) 
             ... speed dynamics ...
                    F_X(x(3),x(4),x(5),x(6))/m + x(4)*x(5);...  \dot vx = FX/m + vy*r
                    F_Y(x(3),x(4),x(5),x(6))/m - x(3)*x(5);...  \dot vx = FY/m - vx*r
                    M_Z(x(3),x(4),x(5),x(6))/Izz;...            \dot r = MZ/Izz
             ... steering angle
                    u(1);...    \detla = \int ddelta
             ... vehicle orientation (yaw)
                    x(5);...    \dot \psi = r
                    ]; 

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

% ---- obstacle ----
obst = @(x,y) -((x-Xa)/R1).^pow -(y/R2).^pow + 1; % obstacle 
opti.subject_to(obst(xpos,ypos)<=0); % ensure that we are far from the elipse

% ---- boundary conditions --------
% define all the initial values here
% position boundry conditions (start)
opti.subject_to(xpos(1)==Xsrt);        
opti.subject_to(ypos(1)==Ystrt);       
% position boundry conditions (finish)
opti.subject_to(xpos(N+1)==Xend); 
opti.subject_to(ypos(N+1)==Yfin);      
% speed boundry conditions (start)
opti.subject_to(vx(1)== Vxstart);   
opti.subject_to(vy(1)==0);          
% % speed boundry conditions (finish)
% opti.subject_to(vy(N+1)==0);      
% % yaw rate boundry conditions (start anmd finish)
% opti.subject_to(r(1)==0);         
% opti.subject_to(r(N+1)==0);           
% % steering angle and rate boundry conditions (start)
% opti.subject_to(delta(1)==0);           
% % steering angle and rate boundry conditions (finish)
% opti.subject_to(delta(N+1)==0);         
% opti.subject_to(ddelta(1)==0);         
% vehicle yaw boundry conditions (start and finish)
opti.subject_to(psi(1)==0);          % start with vehcile orientation ...
% opti.subject_to(psi(N+1)==0);          % finish with vehcile orientation ...

% ---- misc. constraints  ----------
% opti.subject_to(0<=xpos); % xpos must be positive
opti.subject_to(T>=0); % Time must be positive
% opti.subject_to(vx>0); % vx not be equal 0
% opti.subject_to(ypos<=Ymax); % ypos must be positive and under ymax
opti.subject_to(-deltamax<=delta<=deltamax);    % steering angle limit
opti.subject_to(-ddeltamax<=ddelta<=ddeltamax); % steering rate limit

% ---- initial values for solver ---
% define all the initial values here 
% speed initial values 
opti.set_initial(vx, Vxstart);
% opti.set_initial(vy, 0);
% yaw rate initial values 
opti.set_initial(r, 0);
% steering angle and rate initial values 
opti.set_initial(delta, 0);
opti.set_initial(ddelta, 0);
% time initial value
opti.set_initial(T, 1);

% ---- maximum iterations ----
p_opts = struct('expand',true);
s_opts = struct('max_iter',10000);

% ---- solve NLP              ------
opti.solver('ipopt',p_opts,s_opts); % set numerical backend
sol = opti.solve();   % actual solve

%% debuggin plots
prob1.x = opti.debug.value(xpos); prob1.y = opti.debug.value(ypos);
prob1.vx = opti.debug.value(vx); prob1.vy = opti.debug.value(vy);
prob1.r = opti.debug.value(r); % prob1.Fyr = opti.debug.value(Fyr);
prob1.psi = opti.debug.value(psi);
prob1.delta = opti.debug.value(delta);
prob1.ddelta = opti.debug.value(ddelta);
prob1.Fyf = Fyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.Fyr = Fyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.alphaf = alphaf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.alphar = alphar(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.vxf = vxf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.vxr = vxr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.vyf = vyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.vyr = vyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.F_X = F_X(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.F_Y = F_Y(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.M_Z = M_Z(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
% prob1.Fyf = opti.debug.value(Fyf); prob1.Fyr = opti.debug.value(Fyr);
prob1.topt = opti.debug.value(T); prob1.tvec = linspace(0,opti.debug.value(T),N+1);

%% post solution plots
prob1.x = sol.value(xpos); prob1.y = sol.value(ypos);
prob1.vx = sol.value(vx); prob1.vy = sol.value(vy);
prob1.r = opti.debug.value(r); % prob1.Fyr = opti.debug.value(Fyr);
prob1.psi = opti.debug.value(psi);
prob1.delta = sol.value(delta);
prob1.ddelta = sol.value(ddelta);
prob1.Fyf = Fyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.Fyr = Fyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.alphaf = alphaf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.alphar = alphar(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.vxf = vxf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.vxr = vxr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.vyf = vyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.vyr = vyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.F_X = F_X(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.F_Y = F_Y(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
prob1.M_Z = M_Z(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end));
% prob1.Fyf = sol.value(Fyf); prob1.Fyr = sol.value(Fyr);
prob1.topt = sol.value(T); prob1.tvec = linspace(0,sol.value(T),N+1);

%% plotting 

figno = 10;
figure(figno); clf; figno = figno + 1;

plot(prob1.x,prob1.y,'Color','b'); hold on;
fimplicit(@(x,y) obst(x,y),[0 Xend 0 4],'Color','r');
set(gca,'YGrid','on','GridLineStyle','--','GridColor','k');  
ylabel('y [m]','Interpreter','latex');
xlabel('x [m]','Interpreter','latex');


t = prob1.tvec;

for ii = 0:1:round(t(end))
    xi = interp1(t,prob1.x,ii); 
    yi = interp1(t,prob1.y,ii);
    
    yaw = interp1(t,prob1.psi,ii); % yaw at time instant t
    
    [xval, yval] = VehicleShapeNew(xi,yi,yaw,lf+lr,0);

    plot(xval, yval, 'b-','LineWidth',4); % Plotting line segment
end

hold off; xlim([0 inf])

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

figure(figno); clf;

tiledlayout('flow');

nexttile;
plot(prob1.tvec,prob1.vx*3.6); box off;
ylabel('$v_x$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,prob1.vy*3.6); box off;
ylabel('$v_y$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6); box off;
ylabel('$v$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,rad2deg(prob1.r)); box off;
ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,rad2deg(wrapToPi(prob1.psi))); box off;
ylabel('$\psi$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,rad2deg(prob1.delta)); box off;
ylabel('$\delta$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec(2:end),rad2deg(prob1.ddelta)); box off;
ylabel('$\dot \delta$ [deg/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec(2:end),prob1.Fyf); box off;
ylabel('$F_{y(f)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dyf,'LineStyle','--','DisplayName','Fyf (lim)')
yline(-Dyf,'LineStyle','--','DisplayName','Fyf (lim)')

nexttile;
plot(prob1.tvec(2:end),prob1.Fyr); box off;
ylabel('$F_{y(r)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dyr,'LineStyle','--','DisplayName','Fyf (lim)')
yline(-Dyr,'LineStyle','--','DisplayName','Fyf (lim)')

nexttile;
plot(prob1.tvec(2:end),rad2deg([prob1.alphaf; prob1.alphar])); box off;
ylabel('$\alpha_f$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec(2:end),[prob1.vxf; prob1.vxr]*3.6); box off;
ylabel('$v_{x,i}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec(2:end),[prob1.vyf; prob1.vyr]*3.6); box off;
ylabel('$v_{y,i}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec(2:end),prob1.F_X); box off;
ylabel('$F_{X}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec(2:end),prob1.F_Y); box off;
ylabel('$F_{Y}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec(2:end),prob1.M_Z); box off;
ylabel('$M_{Z}$ [Nm]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%% functions and definitions

function [x, y] = VehicleShapeNew(x, y, theta, l, w)        
    box = [x-l/2, y+w/2; x+l/2, y+w/2; x+l/2, y-w/2; x-l/2, y-w/2];    
    box_matrix = box - repmat([x, y], size(box, 1), 1);    
    theta = -theta;    
    rota_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];    
    new = box_matrix * rota_matrix + repmat([x, y], size(box, 1), 1);        
    x = [new(1,1), new(2,1), new(3,1), new(4,1), new(1,1)];    
    y = [new(1,2), new(2,2), new(3,2), new(4,2), new(1,2)];
end