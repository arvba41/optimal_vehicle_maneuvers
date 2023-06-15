clear all; clc;

% load('ST_mov_sol','prob1');
% load('ST_mov_dbg','prob1');

%% ---- model parameterization ----
% Pacejka Magic Formula parameters
params.mux = [1.06 1.07];
params.Bx = [12 11.5];
params.Cx = [1.8 1.8];
params.Ex = [0.313 0.3];
params.muy = [0.885 0.911];
params.By = [10.7 11.30];
params.Cy = [1.07 1.07];
params.Ey = [-2.14 -1.97];
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
%%%% interesting solution
% Xfin    = 6.35+2+30; % center (calculated from A = 34, and w_car + C)
Xfin    = 6.35+2+20; % center (calculated from A = 34, and w_car + C)
% (original)
% Xfin    = 6.35+2+32; % center (calculated from A = 34, and w_car + C)
Xend    = Xfin;
Ystrt   = 0;
Yfin    = 0;

% obstacle parameters
Xa = Xfin;  % center of the obsticle 
Owdt = 6.35+2; % width (w_car + C)
R1 = Owdt;     
Ohgt = 1.7/2 + 2.5; % height (wcar/2 + max(8.5,x), x is any real number)
% ((works with only 100 Npts))
% Ohgt = 1.7/2 + 2.7; % height (wcar/2 + max(8.5,x), x is any real number) 
% (original)
% Ohgt = 1.7/2 + 13.25; % height (wcar/2 + max(8.5,x), x is any real number) 
R2 = Ohgt;  
Osrp = 8;   % squareness of the obsticle 
pow = 4;    % squareness of the obsticle 
Pwid = 0;   % width of the path

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;
% Ymax = 3; % upper limit 

Vxstart = 20; % 72kmph


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
U = opti.variable(2,N);   % control trajectory (steering rate and front wheel force)
ddelta  = U(1,:);
Fxf     = U(2,:);
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
vxf = @(vx,vy,r,delta,Fxf) vx.*cos(delta) + sin(delta).*(vy + lf.*r); 
vyf = @(vx,vy,r,delta,Fxf) cos(delta).*(vy + lf.*r) - vx.*sin(delta);
vxr = @(vx,vy,r,delta,Fxf) vx;
vyr = @(vx,vy,r,delta,Fxf) vy + lr.*r;
% ---- determining the lateral slips ----
alphaf = @(vx,vy,r,delta,Fxf) -atan(vyf(vx,vy,r,delta,Fxf)./vxf(vx,vy,r,delta,Fxf));
alphar = @(vx,vy,r,delta,Fxf) -atan(vyr(vx,vy,r,delta,Fxf)./vxr(vx,vy,r,delta,Fxf));
% ---- determining the lateral forces on the wheels from Fy ----
% maximum forces
Fzf = m*g*lr/(lf+lr); 
Fzr = m*g*lf/(lf+lr);
Dxf = muxf.*Fzf; Dxr = muxr.*Fzr;
Dyf = muyf.*Fzf; Dyr = muyr.*Fzr;
Fy0f = @(vx,vy,r,delta,Fxf) Dyf.*sin(Cyf.*atan(Byf.*alphaf(vx,vy,r,delta,Fxf)-Eyf.*(Byf.*alphaf(vx,vy,r,delta,Fxf) - atan(Byf.*alphaf(vx,vy,r,delta,Fxf)))));
Fy0r = @(vx,vy,r,delta,Fxf) Dyr.*sin(Cyr.*atan(Byr.*alphar(vx,vy,r,delta,Fxf)-Eyr.*(Byr.*alphar(vx,vy,r,delta,Fxf) - atan(Byr.*alphar(vx,vy,r,delta,Fxf)))));
Fyf = @(vx,vy,r,delta,Fxf) Fy0f(vx,vy,r,delta,Fxf);
Fyr = @(vx,vy,r,delta,Fxf) Fy0r(vx,vy,r,delta,Fxf);
% ---- the total forces and moments acting on the vehcile ----
F_X = @(vx,vy,r,delta,Fxf)  cos(delta).*Fxf ...
                        - sin(delta).*Fyf(vx,vy,r,delta,Fxf);  
F_Y = @(vx,vy,r,delta,Fxf)  cos(delta).*Fyf(vx,vy,r,delta,Fxf) ...
                        + sin(delta).*Fxf ...
                        + Fyr(vx,vy,r,delta,Fxf);
M_Z = @(vx,vy,r,delta,Fxf)  lf*cos(delta).*Fyf(vx,vy,r,delta,Fxf) ...
                        + lf*sin(delta).*Fxf ...
                        - lr*Fyr(vx,vy,r,delta,Fxf);

% ---- ODE ----
f = @(x,u) [ ... posistion dynamics ...
                    x(3)*cos(x(7)) - x(4)*sin(x(7)); ...    \dot x = vx*cos(psi) - vy*sin(psi) 
                    x(3)*sin(x(7)) + x(4)*cos(x(7)); ...    \dot y = vx*sin(psi) - vy*cos(psi) 
             ... speed dynamics ...
                    F_X(x(3),x(4),x(5),x(6),u(2))/m + x(4)*x(5);...  \dot vx = FX/m + vy*r
                    F_Y(x(3),x(4),x(5),x(6),u(2))/m - x(3)*x(5);...  \dot vx = FY/m - vx*r
                    M_Z(x(3),x(4),x(5),x(6),u(2))/Izz;...            \dot r = MZ/Izz
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
% obst = @(x,y) -((x-Xa)/R1).^pow -(y/R2).^pow + 1; % obstacle 
% opti.subject_to(obst(xpos,ypos)<=0); % ensure that we are far from the elipse
% defining smooth heaviside functions for step-up and step-down
Hs_p = @(x,cen,xofc,fac) 0.5*(1 + tanh(2*fac*pi*(x-xofc)/cen));
Hs_n = @(x,cen,xofc,fac) 0.5*(1 + tanh(2*fac*pi*(xofc-x)/cen));
% pulse function 
Hs_pulse = @(x,cen,wdh,fac) (Hs_p(x,cen,cen - wdh,fac).*Hs_n(x,cen,cen + wdh,fac)) ;
% obsticle
obst = @(x,cen,wdh,fac,hgt,ofc) hgt*Hs_pulse(x,cen,wdh,fac) + ofc;
% opti.subject_to(obst(xpos,Xfin,Owdt,Osrp,Ohgt,-2*Pwid)<= ypos <= obst(xpos,Xfin,Owdt+Pwid,Osrp,Ohgt,-Pwid)); % ensure that we are far from the elipse
opti.subject_to(obst(xpos,Xfin,Owdt,Osrp,Ohgt,Pwid)<= ypos); % ensure that we are far from the elipse

% ---- boundary conditions --------
% define all the initial values here
% position boundry conditions (start)
opti.subject_to(xpos(1)==Xsrt);        
opti.subject_to(ypos(1)==Ystrt);       
% position boundry conditions (finish)
opti.subject_to(xpos(N+1)==Xend); 
% opti.subject_to(ypos(N+1)==Yfin);      
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
% opti.subject_to(0<=ypos); % ypos must be positive 

%%%% at the limit constraint
% opti.subject_to(F_X(vx(2:end),vy(2:end),r(2:end),delta(2:end),Fxf) == 

% ---- steering constrains ----
opti.subject_to(-deltamax<=delta<=deltamax);    % steering angle limit
opti.subject_to(-ddeltamax<=ddelta<=ddeltamax); % steering rate limit
% ---- force constrains ----
% opti.subject_to(-Dxf*0.98<=Fxf<=0);  
opti.subject_to(Fxf == -Dxf);    

% % % % % % ---- initial values for solver ---
% define all the initial values here 
% speed initial values 
opti.set_initial(vx, Vxstart);
% opti.set_initial(vy, 0);
% yaw rate initial values 
opti.set_initial(r, 0);
% steering angle and rate initial values 
opti.set_initial(delta, 0);
opti.set_initial(ddelta, 0);
% yaw initial values 
opti.set_initial(psi, 0);
% time initial value
opti.set_initial(T, 1);


% % % % % % ---- initial values for solver ---
% % % % % % define all the initial values here 
% % % % % % position initial values 
% % % % % opti.set_initial(xpos, prob1.x);
% % % % % opti.set_initial(ypos, prob1.y);
% % % % % % speed initial values 
% % % % % opti.set_initial(vx, prob1.vx);
% % % % % opti.set_initial(vy, prob1.vy);
% % % % % % yaw rate initial values 
% % % % % opti.set_initial(r, prob1.r);
% % % % % % steering angle and rate initial values 
% % % % % opti.set_initial(delta, prob1.delta);
% % % % % opti.set_initial(ddelta, prob1.ddelta);
% % % % % % yaw initial values 
% % % % % opti.set_initial(psi, prob1.psi);
% % % % % % time initial value
% % % % % opti.set_initial(T, prob1.topt);
% % % % % % fources initial value
% % % % % opti.set_initial(Fxf, prob1.Fxf);

% ---- maximum iterations ----
p_opts = struct('expand',true);
s_opts = struct('max_iter',3000);

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
prob1.Fyf = Fyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.Fyr = Fyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.alphaf = alphaf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.alphar = alphar(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.vxf = vxf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.vxr = vxr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.vyf = vyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.vyr = vyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.F_X = F_X(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.F_Y = F_Y(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.M_Z = M_Z(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
% prob1.Fyf = opti.debug.value(Fyf); prob1.Fyr = opti.debug.value(Fyr);
prob1.topt = opti.debug.value(T); prob1.tvec = linspace(0,opti.debug.value(T),N+1);

save('ST_mov_dbg','prob1');

%% post solution plots
prob1.x = sol.value(xpos); prob1.y = sol.value(ypos);
prob1.vx = sol.value(vx); prob1.vy = sol.value(vy);
prob1.r = opti.debug.value(r); % prob1.Fyr = opti.debug.value(Fyr);
prob1.psi = opti.debug.value(psi);
prob1.delta = sol.value(delta);
prob1.ddelta = sol.value(ddelta);
prob1.Fxf = sol.value(Fxf);
prob1.Fyf = Fyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.Fyr = Fyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.alphaf = alphaf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.alphar = alphar(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.vxf = vxf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.vxr = vxr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.vyf = vyf(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.vyr = vyr(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.F_X = F_X(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.F_Y = F_Y(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
prob1.M_Z = M_Z(prob1.vx(2:end),prob1.vy(2:end),prob1.r(2:end),prob1.delta(2:end),prob1.Fxf);
% prob1.Fyf = sol.value(Fyf); prob1.Fyr = sol.value(Fyr);
prob1.topt = sol.value(T); prob1.tvec = linspace(0,sol.value(T),N+1);

save('ST_mov_sol','prob1');

%% plotting 

figno = 10;
figure(figno); clf; figno = figno + 1;

plot(prob1.x,prob1.y,'Color','b'); hold on;
% fimplicit(@(x,y) obst(x,y),[0 Xend 0 4],'Color','r');
% plot(prob1.x,obst(prob1.x,Xfin,Owdt,Osrp,Ohgt,Pwid),'Color','r');
fplot(@(x) obst(x,Xfin,Owdt,Osrp,Ohgt,Pwid),[0 Xend],'--','Color','k');
% plot(prob1.x,obst(prob1.x,Xfin,Owdt+Pwid,Osrp,Ohgt,-Pwid),'Color','r');
% obst(xpos,Xfin,Owdt,Osrp,Ohgt,-2*Pwid)
set(gca,'YGrid','on','GridLineStyle','--','GridColor','k');  
ylabel('y [m]','Interpreter','latex');
xlabel('x [m]','Interpreter','latex');


t = prob1.tvec;

for ii = 0:0.5:round(t(end))
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
plot(prob1.tvec(2:end),prob1.Fxf); box off;
ylabel('$F_{x(f)}$ [Nm]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dxf,'LineStyle','--','DisplayName','Fxf (lim)')
yline(-Dxf,'LineStyle','--','DisplayName','Fxf (lim)')

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

%% post processing 

% ---- determing the theta from the OCP
% finding intersection point between vehciel and obstacle 
yobst_sim = obst(prob1.x,Xfin,Owdt,Osrp,Ohgt,Pwid);
ypos_sim = prob1.y;
xpos_sim = prob1.x;

idx_int = find(round(yobst_sim,4)== round(ypos_sim,4));
idx_int(1) = []; % ignoting the first element

if length(idx_int) ~= 1
    theta_sim = atan(diff(ypos_sim(idx_int))/diff(xpos_sim(idx_int)));
else
    theta_sim = atan((ypos_sim(idx_int+1) - ypos_sim(idx_int-1))/(xpos_sim(idx_int+1) - xpos_sim(idx_int-1)));
    % approximates derivatives at idx_int using central differences
end

% ---- determing the theta analytically
xpos_f_th = mean(xpos_sim(idx_int));
ypos_f_th = mean(ypos_sim(idx_int));

gamma_th = atan(ypos_f_th/xpos_f_th);

theta_th = (gamma_th + asin(3*sin(gamma_th)))/2;

disp(['Theoretical value of \theta from paper IV: ', num2str(rad2deg(theta_th)), ' deg']);
disp(['Value of \theta from optimal obstacle avoidance of ST model: ', num2str(rad2deg(theta_sim)), ' deg']);

% Earth fixed forces
prob1.FPx = prob1.F_X.*cos(prob1.psi(2:end)) - prob1.F_Y.*sin(prob1.psi(2:end));
prob1.FPy = prob1.F_X.*sin(prob1.psi(2:end)) + prob1.F_Y.*cos(prob1.psi(2:end));
prob1.vPx = prob1.vx.*cos(prob1.psi) - prob1.vy.*sin(prob1.psi);
prob1.vPy = prob1.vx.*sin(prob1.psi) + prob1.vy.*cos(prob1.psi);
prob1.FPxmax = (Dxf+Dxr);
prob1.FPymax = (Dyf+Dyr);

figure(12); clf
tiledlayout('flow');

ax(1) = nexttile;
plot(prob1.tvec(2:end),prob1.FPx); hold on;
ylabel('$F_{P,x}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(-prob1.FPxmax,'--');

ax(2) = nexttile;
plot(prob1.tvec(2:end),prob1.FPy);
ylabel('$F_{P,y}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(prob1.FPymax,'--');

ax(3) = nexttile;
plot(prob1.tvec(2:end),sqrt(prob1.FPx.^2 + prob1.FPy.^2));
ylabel('$F_{P,y}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(sqrt(prob1.FPxmax.^2 + prob1.FPymax.^2),'--');

ax(4) = nexttile;
plot(prob1.tvec,prob1.vPx); hold on;
ylabel('$v_{P,x}$ [m/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
% yline(-prob1.FPxmax,'--');

ax(5) = nexttile;
plot(prob1.tvec,prob1.vPy);
ylabel('$v_{P,y}$ [m/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
% yline(prob1.FPymax,'--');

ax(6) = nexttile;
plot(prob1.tvec,sqrt(prob1.vPx.^2 + prob1.vPy.^2));
ylabel('$v_{P}$ [m/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
% yline(sqrt(prob1.FPxmax.^2 + prob1.FPymax.^2),'--');

linkaxes(ax,'x'); clear ax;

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%% particle model (forced optimal trajectories)

%%% using YOP 
yops Times: t t0 tf % Parsed by position: t, t0, tf
yops States: xpos ypos vx vy % position, speed
yops Control: Fx Fy % acceleration

x = [xpos; ypos; vx; vy];
u = [Fx; Fy];

% optimal control problem
ocp = yop.ocp('optimal_avoidance_with_particle_model');

% constrains
umax = 0.8*params.g*params.m;
ocp.st((Fx.^2 + Fy.^2) == umax^2);
ocp.st(der(x) == smp_veh_mdl(x,u,params)); 
% ocp.st(Fx <= 0);
% ocp.st(Fy >= 0);
ocp.st(Fx == -0.8*params.g*params.m*sin(theta_th));
ocp.st(Fy == 0.8*params.g*params.m*cos(theta_th));

% onstacle 
% hxt = 0.1;
% ocp.st(obst(xpos,Xfin,Owdt,Osrp,Ohgt+hxt,Pwid)<= ypos);
% ocp.st(vx>=0);
% ocp.st(ypos>=0);

% initial conditrions
ocp.st(0 == t0 <= tf); 
ocp.st(x(t0) == [0; 0; 20; 0]);
ocp.st(xpos(tf) == xpos_f_th); 
% ocp.st(ypos(tf) == 0);

% cost function 
ocp.min(tf);

% solving 
sol = ocp.solve;
% sol = ocp.solve('guess',sol);

%% Yop plots
figure(13); clf; 

tiledlayout('flow');

ax(1) = nexttile([1 2]);
sol.plot(xpos,ypos); hold on;
ylabel('$y$ [m]','Interpreter','latex');
xlabel('$x$ [m]','Interpreter','latex');
fplot(@(x) obst(x,Xfin,Owdt,Osrp,Ohgt,Pwid),[0 Xend],'--','Color','k');

ax(2) = nexttile;
sol.plot(t,vx);
ylabel('$v_x$ [m/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

ax(3) = nexttile;
sol.plot(t,vy);
ylabel('$v_y$ [m/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

ax(4) = nexttile;
sol.plot(t,Fx);
ylabel('$F_x$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(umax,'--');

ax(5) = nexttile;
sol.plot(t,Fy);
ylabel('$F_y$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(umax,'--');

linkaxes(ax(2:end),'x');

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