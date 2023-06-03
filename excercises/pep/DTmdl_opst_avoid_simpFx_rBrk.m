clear all; clc;

% load('prevsol_STMdl___rBrk.mat');

% load('prevsol_STMdl___rBrk_temp.mat');

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
params.w = 0.8;
params.g = 9.82;
params.Izz = 3900; % 25% increase in inertia (case 1 with 50m and 50kph) %% 10% increase in inertia 
params.Iw = 4;
params.Re = 0.3;

% start and final distance
Xsrt    = 0;
Xfin    = 50/2;
Xend    = 2*Xfin - Xsrt;
Ystrt   = 1;
Yfin    = 1;

% obstacle parameters
Xa = Xfin; % Xfin % center of the obsticle 
Owdt = 5;   % width of the obsticle 
Owdty = 5;   % width of the obsticle 
R1 = 2;% Owdt;     
Ohgt = 50;  % height of the obsticle 
R2 = Ohgt;  
Osrp = 8;   % squareness of the obsticle 
pow = 4;    % squareness of the obsticle 
Pwid = -5;   % width of the path

% % obstacle parameters ---- heavyside
% Xa = Xfin;  % center of the obsticle 
% Owdt = 5;   % width of the obsticle 
% R1 = Owdt;     
% Ohgt = 5;  % height of the obsticle 
% R2 = Ohgt;  
% Osrp = 8;   % squareness of the obsticle 
% pow = 4;    % squareness of the obsticle 
% Pwid = 0;   % width of the path

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;
% Ymax = 3; % upper limit 

Vxstart = 30/3.6;

% time constqant for the force 
params.TdFxf = 0.2;
params.TdFxr = 0.2;


%% Car Avoiding an obstacle
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

N = 60;                 % number of control intervals

opti = casadi.Opti();   % Optimization problem

% ---- decision variables ---------
X = opti.variable(9,N+1); % state trajectory
xpos    = X(1,:);
ypos    = X(2,:);
vx      = X(3,:);
vy      = X(4,:);
r       = X(5,:);
delta   = X(6,:);
psi     = X(7,:);
Fxf     = X(8,:);
Fxr     = X(9,:);
U = opti.variable(3,N);   % control trajectory (steering rate and front wheel force)
ddelta  = U(1,:);
Fxfval  = U(2,:);
Fxrval  = U(3,:);
T = opti.variable();      % final time

betax = opti.variable();  % approximation parameters
betay = opti.variable();  % approximation parameters

% ---- objective          ---------
% J = (T + 0.5*(betax + betay));
J = T;
opti.minimize(J);         % minimize time

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
% obst = @(x,y) -((x-Xa)/R1).^pow -(y/R2).^pow + 1; % obstacle 
% opti.subject_to(obst(xpos,ypos)<=0); % ensure that we are far from the elipse
% --- sigmoidal definition of obstacle 
% % defining smooth heaviside functions for step-up and step-down
% Hs_p = @(x,cen,xofc,fac) 0.5*(1 + tanh(2*fac*pi*(x-xofc)/cen));
% Hs_n = @(x,cen,xofc,fac) 0.5*(1 + tanh(2*fac*pi*(xofc-x)/cen));
% % pulse function 
% Hs_pulse = @(x,cen,wdh,fac) (Hs_p(x,cen,cen - wdh,fac).*Hs_n(x,cen,cen + wdh,fac)) ;
% % obsticle
% obst = @(x,cen,wdh,fac,hgt,ofc) hgt*Hs_pulse(x,cen,wdh,fac) + ofc;
% % opti.subject_to(obst(xpos,Xfin,Owdt,Osrp,Ohgt,-2*Pwid)<= ypos <= obst(xpos,Xfin,Owdt+Pwid,Osrp,Ohgt,-Pwid)); % ensure that we are far from the elipse
% opti.subject_to(obst(xpos,Xfin,Owdt,Osrp,Ohgt,Pwid)<= ypos); % ensure that we are far from the elipse
%----------- elipses 
% path_in = @(x,y) ((x-Xa)/R1).^pow + (y/R2).^pow;
% path_out = @(x,y) ((x-Xa)/(R1+Owdt)).^pow + (y/(R2+Owdty)).^pow;
% opti.subject_to(path_in(xpos,ypos) >= 1);
% opti.subject_to(path_out(xpos,ypos) <= 1);

% opti.subject_to((Xend - betax) <= xpos(N+1) <= (Xend + betax));
% opti.subject_to((Yfin - betay) <= ypos(N+1) <= (Yfin + betay));
% 
% opti.subject_to(0 <= betax <= 1);
% opti.subject_to(0 <= betay <= 1);

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
opti.subject_to(r(1)==0);         
% opti.subject_to(r(N+1)==0);           
% % steering angle and rate boundry conditions (start)
opti.subject_to(delta(1)==0);           
% opti.subject_to(ddelta(1)==0);         
% % steering angle and rate boundry conditions (finish)
opti.subject_to(delta(N+1)==0);         
% opti.subject_to(ddelta(N+1)==0);      
% vehicle yaw boundry conditions (start and finish)
opti.subject_to(psi(1)==0);  
% opti.subject_to(psi(N+1)==0); 
% fource initial conditions
opti.subject_to(Fxf(1)==0);
opti.subject_to(Fxr(1)==0);
% fource input initial conditions
opti.subject_to(Fxfval(1)==0);
opti.subject_to(Fxrval(1)==0);


% ---- misc. constraints  ----------
% opti.subject_to(0<=xpos); % xpos must be positive
opti.subject_to(T>=0); % Time must be positive
opti.subject_to(vx>0); % vx not be equal 0
% opti.subject_to(ypos<=Ymax); % ypos must be positive and under ymax
% opti.subject_to(0<=ypos); % ypos must be positive 

% ---- steering constrains ----
opti.subject_to(-deltamax<=delta<=deltamax);    % steering angle limit
opti.subject_to(-ddeltamax<=ddelta<=ddeltamax); % steering rate limit
% ---- force constrains ----
opti.subject_to(-Dxf/2<=Fxfval<=Dxf/2);    
opti.subject_to(-Dxf/2<=Fxrval<=0);      % rear wheel braking only

% % % % % ---- initial values for solver ---
% % % % % define all the initial values here 
% % % % % position initial values 
% % % % opti.set_initial(xpos, prob1.x);
% % % % opti.set_initial(ypos, prob1.y);
% % % % % speed initial values 
% % % % opti.set_initial(vx, prob1.vx);
% % % % opti.set_initial(vy, prob1.vy);
% % % % % yaw rate initial values 
% % % % opti.set_initial(r, prob1.r);
% % % % % steering angle and rate initial values 
% % % % opti.set_initial(delta, prob1.delta);
% % % % opti.set_initial(ddelta, prob1.ddelta);
% % % % % yaw initial values 
% % % % opti.set_initial(psi, prob1.psi);
% % % % % time initial value
% % % % opti.set_initial(T, prob1.topt);
% % % % % fources initial value
% % % % opti.set_initial(Fxfval, prob1.Fxfval);
% % % % opti.set_initial(Fxrval, prob1.Fxrval);

% ---- initial values for solver ---
% Very first time 
% position initial values 
% opti.set_initial(x, prob1.x);
% opti.set_initial(y, prob1.y);
% speed initial values 
opti.set_initial(vx, Vxstart);
% opti.set_initial(vy, prob1.vy);
% yaw rate initial values 
opti.set_initial(r, 0);
% steering angle and rate initial values 
opti.set_initial(delta, 0);
opti.set_initial(ddelta, 0);
% yaw initial values 
opti.set_initial(psi, 0);
% time initial value
opti.set_initial(T, 10);
% fources initial value
opti.set_initial(Fxfval, 0);
opti.set_initial(Fxrval, 0);

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
prob1.Fxf = opti.debug.value(Fxf);
prob1.Fxr = opti.debug.value(Fxr);
prob1.Fyf = Fyf(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.Fyr = Fyr(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.alphaf = alphaf(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.alphar = alphar(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.vxf = vxf(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.vxr = vxr(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.vyf = vyf(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.vyr = vyr(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.F_X = F_X(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.F_Y = F_Y(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.M_Z = M_Z(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
% prob1.Fyf = opti.debug.value(Fyf); prob1.Fyr = opti.debug.value(Fyr);
prob1.topt = opti.debug.value(T); prob1.tvec = linspace(0,opti.debug.value(T),N+1);

%%%% NaN error debugging 
opti.debug.g_describe(525)
opti.debug.x_describe(525)

save('prevsol_STMdl___rBrk_temp.mat','prob1')
%% post solution plots
prob1.x = sol.value(xpos); prob1.y = sol.value(ypos);
prob1.vx = sol.value(vx); prob1.vy = sol.value(vy);
prob1.r = opti.debug.value(r); % prob1.Fyr = opti.debug.value(Fyr);
prob1.psi = opti.debug.value(psi);
prob1.delta = sol.value(delta);
prob1.ddelta = sol.value(ddelta);
prob1.Fxf = sol.value(Fxf);
prob1.Fxr = sol.value(Fxr);
prob1.Fxfval = sol.value(Fxfval);
prob1.Fxrval = sol.value(Fxrval);
prob1.Fyf = Fyf(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxf);
prob1.Fyr = Fyr(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.alphaf = alphaf(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.alphar = alphar(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.vxf = vxf(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.vxr = vxr(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.vyf = vyf(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.vyr = vyr(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.F_X = F_X(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.F_Y = F_Y(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
prob1.M_Z = M_Z(prob1.vx,prob1.vy,prob1.r,prob1.delta,prob1.Fxf,prob1.Fxr);
% prob1.Fyf = sol.value(Fyf); prob1.Fyr = sol.value(Fyr);
prob1.topt = sol.value(T); prob1.tvec = linspace(0,sol.value(T),N+1);

save('prevsol_STMdl___rBrk.mat','prob1');

%% plotting 

figno = 10;
figure(figno); clf; figno = figno + 1;

plot(prob1.x,prob1.y,'Color','b'); hold on;
% fimplicit(@(x,y) obst(x,y),[0 Xend 0 4],'Color','r');
% fplot(@(x) obst(x,Xfin,Owdt,Osrp,Ohgt,Pwid),[0 2*Xfin],'Color','r')
% ---- selip (1/2)
fimplicit(@(x,y) path_in(x,y) - 1 ,[0 2*Xfin 0 Ohgt+Owdty],'Color','r');
fimplicit(@(x,y) path_out(x,y) - 1 ,[0 2*Xfin 0 Ohgt+Owdty],'Color','r')
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
yline(rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
yline(-rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
nexttile;
plot(prob1.tvec(2:end),rad2deg(prob1.ddelta)); box off;
ylabel('$\dot \delta$ [deg/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(rad2deg(ddeltamax),'LineStyle','--','DisplayName','$\dot \delta$ (lim)')
yline(-rad2deg(ddeltamax),'LineStyle','--','DisplayName','$\dot \delta$ (lim)')

nexttile;
plot(prob1.tvec,prob1.Fxf); box off;
ylabel('$F_{x(f)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dxf,'LineStyle','--','DisplayName','Fxf (lim)')
yline(-Dxf,'LineStyle','--','DisplayName','Fxf (lim)')

nexttile;
plot(prob1.tvec,prob1.Fxr); box off;
ylabel('$F_{x(r)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dxf,'LineStyle','--','DisplayName','Fxf (lim)')
yline(-Dxf,'LineStyle','--','DisplayName','Fxf (lim)')

nexttile;
plot(prob1.tvec,prob1.Fyf); box off;
ylabel('$F_{y(f)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dyf,'LineStyle','--','DisplayName','Fyf (lim)')
yline(-Dyf,'LineStyle','--','DisplayName','Fyf (lim)')

nexttile;
plot(prob1.tvec,prob1.Fyr); box off;
ylabel('$F_{y(r)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dyr,'LineStyle','--','DisplayName','Fyf (lim)')
yline(-Dyr,'LineStyle','--','DisplayName','Fyf (lim)')

nexttile;
plot(prob1.tvec,rad2deg([prob1.alphaf; prob1.alphar])); box off;
ylabel('$\alpha_f$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,[prob1.vxf; prob1.vxr]*3.6); box off;
ylabel('$v_{x,i}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,[prob1.vyf; prob1.vyr]*3.6); box off;
ylabel('$v_{y,i}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,prob1.F_X); box off;
ylabel('$F_{X}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,prob1.F_Y); box off;
ylabel('$F_{Y}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,prob1.M_Z); box off;
ylabel('$M_{Z}$ [Nm]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%% functions and definitions

function [x_dot] = DT_model(x,u,params)
%% parameterization 

% ---- states ---- 
vx = x(3);
vy = x(4);
r = x(5);
delta = x(6);
psi = x(7);
Fxf = x(8);
Fxr = x(9);

% ---- inputs ----
ddelta = u(1);
Fxfval = u(2);
Fxrval = u(3);

% parameters 
m   = params.m;
lf  = params.lf; 
lr  = params.lr;
w = params.w;
g   = params.g;
Izz = params.Izz;
Iw  = params.Iw;
Re  = params.Re;
muxf = params.mux(1); muxr = params.mux(2); 
muyf = params.muy(2); muyr = params.muy(2);
Cxf = params.Cx(1); Cxr = params.Cx(2);
Cyf = params.Cy(1); Cyr = params.Cy(2);
Bxf = params.Bx(1); Bxr = params.Bx(2);
Byf = params.By(1); Byr = params.By(2);
Exf = params.Ex(1); Exr = params.Ex(2);
Eyf = params.Ey(1); Eyr = params.Ey(2);
TdFxf = params.TdFxf; TdFxr = params.TdFxr;

%% Equations 
% ---- vehicle lengths and widths ----
lx1 = lf; lx2 = lf; lx3 = -lr; lx4 = -lr;
ly1 = w/2; ly2 = w/2; ly3 = -w/2; ly4 = -w/2;
% ---- individual wheel velocities ----
inter = [cos(delta) sin(delta);-sin(delta) cos(delta)]*([vx; vy] + r*[-ly1; lx1]);
vx1 = inter(1); vy1 = inter(2);
inter = [cos(delta) sin(delta);-sin(delta) cos(delta)]*([vx; vy] + r*[-ly2; lx2]);
vx2 = inter(1); vy2 = inter(2);
inter = [1 0;0 1]*([vx; vy] + r*[-ly3; lx3]);
vx3 = inter(1); vy3 = inter(2);
inter = [1 0;0 1]*([vx; vy] + r*[-ly4; lx4]);
vx4 = inter(1); vy4 = inter(2);
clear inter
% ---- lateral slips ----
alpha1 = -atan(vy1/vx1);
alpha2 = -atan(vy2/vx2);
alpha3 = -atan(vy3/vx3);
alpha4 = -atan(vy4/vx4);
% ---- vertical forces on the wheels ----
Fzf = m*g*lr/(lf+lr)/2; 
Fzr = m*g*lf/(lf+lr)/2;
% ---- force and moment on the center of gravity ----
inter = [1 0; 0 1; -l_y1 l_x1]*[cos(delta) -sin(delta); sin(delta) cos(delta)]*[Fxf; Fy1] + ...
                   [1 0; 0 1; -l_y2 l_x2]*[cos(delta) -sin(delta); sin(delta) cos(delta)]*[Fxf; Fy2] + ...
                   [1 0; 0 1; -l_y3 l_x3]*[1 0; 0 1]*[Fxr; Fy3] + ...
                   [1 0; 0 1; -l_y4 l_x4]*[1 0; 0 1]*[Fxr; Fy4];

FX = inter(1);
FY = inter(2);
MZ = inter(3);
clear inter
% state equations
Xp_dot = vx*cos(psi) - vy*sin(psi);
Yp_dot = vx*sin(psi) + vy*cos(psi);
vx_dot  = FX/m + vy*r;
vy_dot  = FY/m - vx*r;
r_dot   = MZ/Izz;
delta_dot = delta;
psi_dot = r;
Fxf_dot = (Fxfval - Fxf)/TdFxf;
Fxr_dot = (Fxrval - Fxr)/TdFxr;

x_dot = []

end

function [x, y] = VehicleShapeNew(x, y, theta, l, w)        
    box = [x-l/2, y+w/2; x+l/2, y+w/2; x+l/2, y-w/2; x-l/2, y-w/2];    
    box_matrix = box - repmat([x, y], size(box, 1), 1);    
    theta = -theta;    
    rota_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];    
    new = box_matrix * rota_matrix + repmat([x, y], size(box, 1), 1);        
    x = [new(1,1), new(2,1), new(3,1), new(4,1), new(1,1)];    
    y = [new(1,2), new(2,2), new(3,2), new(4,2), new(1,2)];
end

