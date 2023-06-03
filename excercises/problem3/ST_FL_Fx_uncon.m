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
% Linear tire friction model parameters
params.Caf = 103600;
params.Car = 120000;
% ST model parameters
params.m = 2100;
params.lf = 1.3; 
params.lr = 1.5;
params.g = 9.82;
params.Izz = 3900;
params.Iw = 4;
params.Re = 0.3;

% final distance
Xfin = 100/2;

% obstacle parameters
Xa = Xfin;  % center of the obsticle 
R1 = 5;     % width of the obsticle 
R2 = 1.1;   % height of the obsticle 
pow = 4;    % squareness of the obsticle 

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;
Ymax = 3; % upper limit 

Vxstart = 10/3.6;

Ystrt = 1;
Yfin = 1;

Xstrt = 0;
Xend = Xfin*2;

%% Car Avoiding an obstacle
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

% ST model parameters
m = params.m;
lf = params.lf; 
lr = params.lr;
g = params.g;
Izz = params.Izz;
Iw = params.Iw;
Re = params.Re;

N = 60; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(7,N+1); % state trajectory
xpos    = X(1,:);
ypos    = X(2,:);
vx      = X(3,:);
vy      = X(4,:);
r       = X(5,:);
delta   = X(6,:);
psi     = X(7,:);
% omegaf  = X(8,:);
% omegar  = X(9,:);
U = opti.variable(2,N);   % control trajectory (steering rate and throttle)
ddelta  = U(1,:);
Tf      = U(2,:);
T = opti.variable();      % final time

betax = opti.variable();      
betay = opti.variable();      

% ---- objective          ---------
val = T + 0.1*betax^2 + 0.1*betay^2;
opti.minimize(val);         % minimize time

% ---- dynamic constraints -----
% ---- parameters ----
% Calphaf = params.Caf;
% Calphar = params.Car;
muxf = params.mux(1); muxr = params.mux(2); 
muyf = params.muy(2); muyr = params.muy(2);
Cxf = params.Cx(1); Cxr = params.Cx(2);
Cyf = params.Cy(1); Cyr = params.Cy(2);
Bxf = params.Bx(1); Bxr = params.Bx(2);
Byf = params.By(1); Byr = params.By(2);
Exf = params.Ex(1); Exr = params.Ex(2);
Eyf = params.Ey(1); Eyr = params.Ey(2);
% ---- indepemdent wheel velocities ----
vxf = @(vx,vy,r,delta,Tf) vx.*cos(delta) + sin(delta).*(vy + lf.*r); 
vyf = @(vx,vy,r,delta,Tf) cos(delta).*(vy + lf.*r) - vx.*sin(delta);
vxr = @(vx,vy,r,delta,Tf) vx;
vyr = @(vx,vy,r,delta,Tf) vy + lr.*r;
% ---- determining the slips ----
kappaf = @(vx,vy,r,delta,Tf) (Re*omegaf - vxf(vx,vy,r,delta,Tf))./vxf(vx,vy,r,delta,Tf);
kappar = @(vx,vy,r,delta,Tf) (Re*omegar - vxr(vx,vy,r,delta,Tf))./vxr(vx,vy,r,delta,Tf);
alphaf = @(vx,vy,r,delta,Tf) -atan(vyf(vx,vy,r,delta,Tf)./vxf(vx,vy,r,delta,Tf));
alphar = @(vx,vy,r,delta,Tf) -atan(vyr(vx,vy,r,delta,Tf)./vxr(vx,vy,r,delta,Tf));
% ---- determining the lateral forces on the wheels from Fy ----
% Fyf = @(vx,vy,r,delta,Tf) Calphaf*alphaf(vx,vy,r,delta,Tf);
% Fyr = @(vx,vy,r,delta,Tf) Calphar*alphar(vx,vy,r,delta,Tf);
Fzf = m*g*lr/(lf+lr); 
Fzr = m*g*lf/(lf+lr);
Dxf = muxf.*Fzf; Dxr = muxr.*Fzr;
Dyf = muyf.*Fzf; Dyr = muyr.*Fzr;
% Fxf = @(vx,vy,r,delta,Tf) Dxf.*sin(Cxf.*atan(Bxf.*kappaf(vx,vy,r,delta,Tf)-Exf.*(Bxf.*kappaf(vx,vy,r,delta,Tf) - atan(Bxf.*kappaf(vx,vy,r,delta,Tf)))));
% Fxr = @(vx,vy,r,delta,Tf) Dxr.*sin(Cxr.*atan(Bxr.*kappar(vx,vy,r,delta,Tf)-Exr.*(Bxr.*kappar(vx,vy,r,delta,Tf) - atan(Bxr.*kappar(vx,vy,r,delta,Tf)))));
Fxf = @(vx,vy,r,delta,Tf) Tf/Re;
Fxr = @(vx,vy,r,delta,Tf) 0;
Fy0f = @(vx,vy,r,delta,Tf) Dyf.*sin(Cyf.*atan(Byf.*alphaf(vx,vy,r,delta,Tf)-Eyf.*(Byf.*alphaf(vx,vy,r,delta,Tf) - atan(Byf.*alphaf(vx,vy,r,delta,Tf)))));
Fy0r = @(vx,vy,r,delta,Tf) Dyr.*sin(Cyr.*atan(Byr.*alphar(vx,vy,r,delta,Tf)-Eyr.*(Byr.*alphar(vx,vy,r,delta,Tf) - atan(Byr.*alphar(vx,vy,r,delta,Tf)))));
Fyf = @(vx,vy,r,delta,Tf) Fy0f(vx,vy,r,delta,Tf).*sqrt(1 - (Fxf(vx,vy,r,delta,Tf)./Dxf).^2);
Fyr = @(vx,vy,r,delta,Tf) Fy0r(vx,vy,r,delta,Tf).*sqrt(1 - (Fxr(vx,vy,r,delta,Tf)./Dxr).^2);
% ---- the total forces and moments acting on the vehcile ----
F_X = @(vx,vy,r,delta,Tf) cos(delta).*Fxf(vx,vy,r,delta,Tf) - sin(delta).*Fyf(vx,vy,r,delta,Tf) + Fxr(vx,vy,r,delta,Tf);  
F_Y = @(vx,vy,r,delta,Tf) cos(delta).*Fyf(vx,vy,r,delta,Tf) + sin(delta).*Fxf(vx,vy,r,delta,Tf) + Fyr(vx,vy,r,delta,Tf);
M_Z = @(vx,vy,r,delta,Tf) lf*cos(delta).*Fyf(vx,vy,r,delta,Tf) + lf*sin(delta).*Fxf(vx,vy,r,delta,Tf) - lr*Fyr(vx,vy,r,delta,Tf);

% ---- ODE ----
f = @(x,u) [ ... the posistion dynamics ...
                    x(3)*cos(x(7)) - x(4)*sin(x(7)); ...    \dot x = vx*cos(psi) - vy*sin(psi)
                    x(3)*sin(x(7)) + x(4)*cos(x(7)); ...    \dot y = vx*sin(psi) + vy*cos(psi)
             ... the speed dynamics ...
                    F_X(x(3),x(4),x(5),x(6),u(2))/m + x(4)*x(5);...  \dot vx = FX/m + vy*r
                    F_Y(x(3),x(4),x(5),x(6),u(2))/m - x(3)*x(5);...  \dot vx = FY/m - vx*r
                    M_Z(x(3),x(4),x(5),x(6),u(2))/Izz;...            \dot yawrate = MZ/Izz
             ... steering angle
                    u(1);...
             ... vehicle orientation 
                    x(5); ...
             ... wheel dynamics ...
%                     (u(2) - Re*Fxf(x(3),x(4),x(5),x(6),x(8),x(9)))/Iw;... \dot omegaf = (Tf - Re*Fxf)/Iw
%                     -Re*Fxr(x(3),x(4),x(5),x(6),x(8),x(9))/Iw...         \dot omegar = (- Re*Fxr)/Iw
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

% ---- force constraints -----------
% Friction elipse
% Fxfval = Tf/Re;
% Fxrval = Fxr(vx,vy,r,delta,Tf);
% Fyfval = Fyf(vx,vy,r,delta,Tf);
% Fyrval = Fyr(vx,vy,r,delta,Tf);
% % frictions form the parameters
% % muxf = params.mux(1); muxr = params.mux(2); 
% muyf = params.muy(1); muyr = params.muy(2);
% maximum resultant force
% Fzfmax = m*g*lr/(lf+lr); 
% Fzrmax = m*g*lf/(lf+lr);
% Fyfmax = muyf*Fzfmax;
% Fyrmax = muyr*Fzrmax;
Tfmax = Dxf*Re;
opti.subject_to(-Tfmax<=Tf<=Tfmax); % Limiting the imput torquey

% opti.subject_to(-Dxr<=Fxrval<=Dxr); % Force is limited eliptically

% ---- obstacle ----
obst = @(x,y) -((x-Xa)/R1).^pow -(y/R2).^pow + 1; % obstacle 
% opti.subject_to(obst(xpos,ypos)<=0); % ensure that we are far from the elipse

% ---- boundary conditions --------
% define all the initial values here 
opti.subject_to(xpos(1)==Xstrt);        % start at position x 0 ...
opti.subject_to(ypos(1)==Ystrt);        % start at position y 1 ... 
% opti.subject_to(xpos(N+1)==Xend); % finish line at position x ...
% opti.subject_to(ypos(N+1)==Yfin);      % finish line at position y ...
opti.subject_to(vx(1)== Vxstart);     % start with speed x ...
opti.subject_to(vy(1)==0);          % start with speed y ...
% opti.subject_to(vy(N+1)==0);          % finish with speed y ...
opti.subject_to(r(1)==0);           % start with yaw rate ...
% opti.subject_to(r(N+1)==0);           % finish with yaw rate ...
opti.subject_to(delta(1)==0);           % start with yaw rate ...
% opti.subject_to(delta(N+1)==0);         % finish with yaw rate ...
opti.subject_to(ddelta(1)==0);           % start with yaw rate ...
opti.subject_to(psi(1)==0);          % start with vehcile orientation ...
% opti.subject_to(ddelta(N)==0);          % finish with yaw rate ...
% opti.subject_to(omegaf(1)==Vxstart/Re); % start with angular velocity font ...
% opti.subject_to(omegar(1)==Vxstart/Re); % start with angular velocity rear ...
opti.subject_to(Tf(1)==0);           % start with torque...

% ---- misc. constraints  ----------
opti.subject_to(0<=xpos); % xpos must be positive
opti.subject_to(T>=0); % Time must be positive
% opti.subject_to(vx>0); % vx not be equal 0
opti.subject_to(0<=ypos); % ypos must be positive and under ymaxS
opti.subject_to(ypos<=Ymax); % ypos must be positive and under ymaxS
% opti.subject_to(-ddeltamax<=r<=ddeltamax); % steering rate limit

opti.subject_to(-deltamax<=delta<=deltamax); % steering angle limit
opti.subject_to(-ddeltamax<=ddelta<=ddeltamax); % steering rate limit

opti.subject_to(-0.1<=betax<=0.1)
opti.subject_to(-0.1<=betay<=0.1)

opti.subject_to((Xend - betax)<=xpos(N+1)<=(Xend + betax))
opti.subject_to((Yfin - betay)<=ypos(N+1)<=(Yfin + betay))

% ---- initial values for solver ---
% define all the initial values here 
% opti.set_initial(xpos, 0);
% opti.set_initial(ypos, 1);
opti.set_initial(vx, Vxstart);
% opti.set_initial(vy, 0);
% opti.set_initial(r, 0);
% opti.set_initial(delta, 0);
% opti.set_initial(ddelta, 0);
opti.set_initial(T, 1);
opti.set_initial(betax, 0.1);
opti.set_initial(betay, 0.1);

% ---- maximum iterations ----
p_opts = struct('expand',true);
s_opts = struct('max_iter',10000);

% ---- solve NLP              ------
opti.solver('ipopt',p_opts,s_opts); % set numerical backend
sol = opti.solve();   % actual solve

%% debuggin plots
prob1.x = opti.debug.value(xpos); prob1.y = opti.debug.value(ypos);
prob1.vx = opti.debug.value(vx); prob1.vy = opti.debug.value(vy);
prob1.delta = opti.debug.value(delta);
prob1.r = opti.debug.value(r); % prob1.Fyr = opti.debug.value(Fyr);
prob1.psi = opti.debug.value(psi);
prob1.topt = opti.debug.value(T); prob1.tvec = linspace(0,opti.debug.value(T),N+1);
prob1.ddelta = opti.debug.value(ddelta);
prob1.Fyf = Fyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.Fyr = Fyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.alphaf = alphaf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.alphar = alphar(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.vxf = vxf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.vxr = vxr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.vyf = vyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.vyr = vyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.F_X = F_X(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.F_Y = F_Y(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
prob1.M_Z = M_Z(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),opti.debug.value(Tf));
% prob1.Fyf = opti.debug.value(Fyf); prob1.Fyr = opti.debug.value(Fyr);
prob1.topt = opti.debug.value(T); prob1.tvec = linspace(0,opti.debug.value(T),N+1);

figno = 1;
figure(figno); clf; figno = figno + 1;

plot(prob1.x,prob1.y,'x-','Color','b'); hold on;
fimplicit(@(x,y) obst(x,y),[0 100 0 6],'Color','r');
set(gca,'YGrid','on','GridLineStyle','--','GridColor','k');  
ylabel('y [m]','Interpreter','latex');
xlabel('x [m]','Interpreter','latex');


% lf = 0.3; % front length of the vehicle from CoM
% lr = 0.3; % rear length of the vehicle from CoM
% 
% % yaw_vec = cumtrapz(t,prob1.r); % determining the yaw angle
% % tseg = linspace(1,t(end),5); % time steps for line segment
% t = prob1.tvec;
% 
% for ii = 0:0.5:round(t(end))
%     xi = interp1(t,prob1.x,ii); 
%     yi = interp1(t,prob1.y,ii);
%     
%     yaw = interp1(t,prob1.psi,ii); % yaw at time instant t
%     
%     xstart = xi - lr*(cos(yaw)); % x-coordinate of line segment startpoint
%     xend = xi + lf*(cos(yaw)); % x-coordinate of line segment endpoint
%     ystart = yi - lr*(sin(yaw)); % x-coordinate of line segment startpoint
%     yend = yi + lf*(sin(yaw)); % x-coordinate of line segment endpoint
%     
%     plot([xstart, xend], [ystart, yend], 'b-','LineWidth',4); % Plotting line segment
% end

hold off; xlim([0 inf])

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

figure(figno); clf;

tiledlayout('flow');

% nexttile;
% plot(prob1.tvec,prob1.x); box off;
% ylabel('$x$ [m]','Interpreter','latex');
% xlabel('$t$ [s]','Interpreter','latex');
% 
% nexttile;
% plot(prob1.tvec,prob1.y); box off;
% ylabel('$y$ [m]','Interpreter','latex');
% xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,prob1.vx*3.6); box off;
ylabel('$v_x$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,prob1.vy*3.6); box off;
ylabel('$v_y$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,rad2deg(prob1.r)); box off;
ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,rad2deg(mod(prob1.psi,(2*pi)))); box off;
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

% %% plotting
% f = figure(1); clf;
% optim_plot_prob1(prob1);

%% post solution plots
prob1.x = sol.value(xpos); prob1.y = sol.value(ypos);
prob1.vx = sol.value(vx); prob1.vy = sol.value(vy);
prob1.r = opti.debug.value(r); % prob1.Fyr = opti.debug.value(Fyr);
prob1.psi = opti.debug.value(psi);
prob1.delta = sol.value(delta);
prob1.ddelta = sol.value(ddelta);
prob1.Fyf = Fyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.Fyr = Fyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.alphaf = alphaf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.alphar = alphar(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.vxf = vxf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.vxr = vxr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.vyf = vyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.vyr = vyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.F_X = F_X(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.F_Y = F_Y(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
prob1.M_Z = M_Z(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),sol.value(Tf));
% prob1.Fyf = sol.value(Fyf); prob1.Fyr = sol.value(Fyr);
prob1.topt = sol.value(T); prob1.tvec = linspace(0,sol.value(T),N+1);

figno = 10;
figure(figno); clf; figno = figno + 1;

plot(prob1.x,prob1.y,'x-','Color','b'); hold on;
fimplicit(@(x,y) obst(x,y),[0 100 0 6],'Color','r');
set(gca,'YGrid','on','GridLineStyle','--','GridColor','k');  
ylabel('y [m]','Interpreter','latex');
xlabel('x [m]','Interpreter','latex');


% lf = 0.3; % front length of the vehicle from CoM
% lr = 0.3; % rear length of the vehicle from CoM
% 
% % yaw_vec = cumtrapz(t,prob1.r); % determining the yaw angle
% % tseg = linspace(1,t(end),5); % time steps for line segment
% t = prob1.tvec;
% 
% for ii = 0:0.5:round(t(end))
%     xi = interp1(t,prob1.x,ii); 
%     yi = interp1(t,prob1.y,ii);
%     
%     yaw = interp1(t,prob1.psi,ii); % yaw at time instant t
%     
%     xstart = xi - lr*(cos(yaw)); % x-coordinate of line segment startpoint
%     xend = xi + lf*(cos(yaw)); % x-coordinate of line segment endpoint
%     ystart = yi - lr*(sin(yaw)); % x-coordinate of line segment startpoint
%     yend = yi + lf*(sin(yaw)); % x-coordinate of line segment endpoint
%     
%     plot([xstart, xend], [ystart, yend], 'b-','LineWidth',4); % Plotting line segment
% end

hold off; xlim([0 inf])

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

figure(figno); clf;

tiledlayout('flow');

% nexttile;
% plot(prob1.tvec,prob1.x); box off;
% ylabel('$x$ [m]','Interpreter','latex');
% xlabel('$t$ [s]','Interpreter','latex');
% 
% nexttile;
% plot(prob1.tvec,prob1.y); box off;
% ylabel('$y$ [m]','Interpreter','latex');
% xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,prob1.vx*3.6); box off;
ylabel('$v_x$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,prob1.vy*3.6); box off;
ylabel('$v_y$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,rad2deg(prob1.r)); box off;
ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,rad2deg(mod(prob1.psi,(2*pi)))); box off;
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

% %% plotting
% f = figure(1); clf;
% optim_plot_prob1(prob1);
