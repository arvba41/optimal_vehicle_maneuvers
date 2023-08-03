clear all; clc;

%% ---- model parameterization ----

% Pacejka Magic Formula parameters
sel_fric = 2; % selecting tire parameters based on friction
switch sel_fric
    case 1  % Dry ashpalt
        params.mux  = [1.2      1.2     ];
        params.Bx   = [11.7     11.1    ];
        params.Cx   = [1.69     1.69    ];
        params.Ex   = [0.377    0.362   ];
        params.muy  = [0.935    0.961   ];
        params.By   = [8.86     9.30    ];
        params.Cy   = [1.19     1.19    ];
        params.Ey   = [-1.21    -1.11   ];
    case 2  % Wet ashpalt 
        params.mux  = [1.06     1.07    ];
        params.Bx   = [12       11.5    ];
        params.Cx   = [1.80     1.80    ];
        params.Ex   = [0.313    0.300   ];
        params.muy  = [0.885    0.911   ];
        params.By   = [10.7     11.3    ];
        params.Cy   = [1.07     1.07    ];
        params.Ey   = [-2.14    -1.97   ];
    case 3  % Snow ashpalt
        params.mux  = [0.407    0.409   ];
        params.Bx   = [10.2     9.71    ];
        params.Cx   = [1.96     1.96    ];
        params.Ex   = [0.651    0.624   ];
        params.muy  = [0.383    0.394   ];
        params.By   = [19.1     20      ];
        params.Cy   = [0.550    0.550   ];
        params.Ey   = [-2.10    -1.93   ];
    case 4  % Smooth ice
        params.mux  = [0.172    0.173   ];
        params.Bx   = [31.1     29.5    ];
        params.Cx   = [1.77     1.77    ];
        params.Ex   = [0.710    0.681   ];
        params.muy  = [0.162    0.167   ];
        params.By   = [28.4     30      ];
        params.Cy   = [1.48     1.48    ];
        params.Ey   = [-1.18    -1.08   ];
end
% vehicle parameters
params.m = 2100;
params.lf = 1.3; 
params.lr = 1.5;
params.w = 0.8;
params.g = 9.82;
params.Izz = 3900; % 25% increase in inertia (case 1 with 50m and 50kph) %% 10% increase in inertia 
params.Iw = 4;
params.Re = 0.3;
params.h = 0.5;
    
% start and final distance
Xsrt    = 0;
Xa      = 50;
Xend    = Xa - Xsrt;
Ystrt   = 0;
Yend    = 0;

% obstacle parameters
% Xa is the center of the obstacle
R1  = 2; % width of the obstalce
R2  = 3; % height of the obstalce
n   = 12; % sharpness of the obstacle

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;

Vxstart = 20; % initial velocitty

% time constqant for the force 
params.TdFxf = 0.1;
params.TdFxr = 0.1;

% limiting the search space
Ymax = R2*2.5;
%% Car Avoiding an obstacle
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

N = 100;                % number of control intervals

% determining gamma and theta 
B = R2; 
A = Xa - R1;
gamma = atan(B / A);
theta = (gamma + asin(3 * sin(gamma)))/2;

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
Fz = opti.variable(4,:);  % Forces on the wheels
Fz1 = Fz(1,:);  
Fz2 = Fz(2,:);  
Fz3 = Fz(3,:);  
Fz4 = Fz(4,:);  

epsilon = opti.variable();
Ecy = opti.variable(1,N+1);

% ---- objective          ---------

% J = -(vx(1));         % maximum entry speed
J = T;                  % minimum time
% J = -Ecy(end);        % maximum Fcy or Fy
opti.minimize(J);       % minimize time

% ---- dynamic constraints -----
dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
    
   % Runge-Kutta 4 integration              
   k1 = DT_model(X(:,k),         U(:,k),    params,     Fz(:,k));
   k2 = DT_model(X(:,k)+dt/2*k1, U(:,k),    params,     Fz(:,k));
   k3 = DT_model(X(:,k)+dt/2*k2, U(:,k),    params,     Fz(:,k));
   k4 = DT_model(X(:,k)+dt*k3,   U(:,k),    params,     Fz(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
   
   % get the forces
   [~, ~, Fx, Fy] = DT_model(X(:,k), U(:,k), params,    Fz(:,k));
   
   % 
end

% ---- obstacle ----
% obst = @(x,y) ((x-Xa)/R1).^n + (y/R2).^n;
% opti.subject_to(obst(xpos,ypos) >= 1);

% ---- boundary conditions --------
opti.subject_to(xpos(1)==Xsrt);        
opti.subject_to(ypos(1)==Ystrt);       

opti.subject_to(xpos(N+1)==Xend); 
opti.subject_to(ypos(N+1)==Yend);      

opti.subject_to(vx(1)== Vxstart);   
% opti.subject_to(vx(1)== prob1.vx(1)); 
opti.subject_to(vy(1)==0);          

% opti.subject_to(vy(N+1)==0);      

opti.subject_to(r(1)==0);         
% opti.subject_to(r(N+1)==0);           

opti.subject_to(delta(1)==0);           
opti.subject_to(ddelta(1)==0);         

% opti.subject_to(delta(N+1)==0);         
% opti.subject_to(ddelta(N+1)==0);      

opti.subject_to(psi(1)==0);  
% opti.subject_to(psi(N+1)==0); 

opti.subject_to(Ecy(1)==0);

% opti.subject_to(Fxf(1)==0);
% opti.subject_to(Fxr(1)==0);

% opti.subject_to(Fxfval(1)==0);
% opti.subject_to(Fxrval(1)==0);

% ---- misc. constraints  ----------
% opti.subject_to(0<= T <= 2); % Time must be positive
opti.subject_to(vx >0); % vx not be equal 0
opti.subject_to(0 <= ypos <= Ymax); % reducing the search space
opti.subject_to(0 <= xpos <= Xend); % reducing the search space

% ---- steering constrains ----
opti.subject_to(-deltamax<=delta<=deltamax);    % steering angle limit
opti.subject_to(-ddeltamax<=ddelta<=ddeltamax); % steering rate limit

% ---- force constrains ----
Fzfmax = params.m * params.g * params.lr / (params.lf + params.lr) / 2; 
Fzrmax = params.m * params.g * params.lf / (params.lf + params.lr) / 2; 
Dxf = params.mux(1) * Fzfmax; Dxr = params.mux(2) * Fzrmax;
Dyf = params.muy(1) * Fzfmax; Dyr = params.muy(2) * Fzrmax;
opti.subject_to(-Dxf <= Fxfval <= Dxf);    
opti.subject_to(-Dxr <= Fxrval <= 0);   % rear wheel braking only
opti.subject_to(-Dxf <= Fxf <= Dxf);    
opti.subject_to(-Dxr <= Fxr <= 0);      % rear wheel braking only

% ---- maximizing the force ----
% opti.subject_to(1 <= epsilon <= 1.05);
% opti.subject_to(Fcy == (Dyf*2 + Dyr*2) * 0.98);
% opti.subject_to(Fcx == 0);

% ---- initial values for solver ---
% % % % % % 
% % % % % % opti.set_initial(xpos, prob1.x);
% % % % % % opti.set_initial(ypos, prob1.y);
% % % % % % 
% % % % % % opti.set_initial(vx, prob1.vx);
% % % % % % opti.set_initial(vy, prob1.vy);
% % % % % % 
% % % % % % opti.set_initial(r, prob1.r);
% % % % % % 
% % % % % % opti.set_initial(delta, prob1.delta);
% % % % % % opti.set_initial(ddelta, prob1.ddelta);
% % % % % % 
% % % % % % opti.set_initial(psi, prob1.psi);
% % % % % % 
% % % % % % opti.set_initial(T, prob1.topt);
% % % % % % 
% % % % % % opti.set_initial(Fxfval, prob1.Fxfval);
% % % % % % opti.set_initial(Fxrval, prob1.Fxrval);

% % % % % % % ---- initial values for solver --- (Very first time)
% opti.set_initial(x, prob1.x);
% opti.set_initial(ypos, 0);

opti.set_initial(vx, 30);
% opti.set_initial(vy, prob1.vy);

opti.set_initial(r, 0);

opti.set_initial(delta, 0);
opti.set_initial(ddelta, 0);

opti.set_initial(psi, 0);

opti.set_initial(T, 1);

opti.set_initial(Ecy, 1e3);
opti.set_initial(Fxf, 0);
opti.set_initial(Fxr, 0);
opti.set_initial(Fxfval, -10);
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
prob1.Fxfval = opti.debug.value(Fxfval);
prob1.Fxrval = opti.debug.value(Fxrval);
prob1.Ecy = opti.debug.value(Ecy);

for ii = 1 : (N+1)
    [~, inter(ii),Fxv(ii),Fyv(ii)] = DT_model(opti.debug.value(X(:,ii)), ...
                            opti.debug.value(U(:,1)), ...
                            params);
    prob1.M_Z(ii) = inter(ii).M_Z;
    prob1.F_Y(ii) = inter(ii).F_Y;
    prob1.F_X(ii) = inter(ii).F_X;
    prob1.vy1(ii) = inter(ii).vy1;
    prob1.vy2(ii) = inter(ii).vy2;
    prob1.vy3(ii) = inter(ii).vy3;
    prob1.vy4(ii) = inter(ii).vy4;
    prob1.vx1(ii) = inter(ii).vx1;
    prob1.vx2(ii) = inter(ii).vx2;
    prob1.vx3(ii) = inter(ii).vx3;
    prob1.vx4(ii) = inter(ii).vx4;
    prob1.alpha1(ii) = inter(ii).alpha1;
    prob1.alpha2(ii) = inter(ii).alpha2;
    prob1.alpha3(ii) = inter(ii).alpha3;
    prob1.alpha4(ii) = inter(ii).alpha4;
    prob1.Fy1(ii) = inter(ii).Fy1;
    prob1.Fy2(ii) = inter(ii).Fy2;
    prob1.Fy3(ii) = inter(ii).Fy3;
    prob1.Fy4(ii) = inter(ii).Fy4;
end

Dxf = params.mux(1) * Fzfmax; Dxr = params.mux(2) * Fzrmax;
Dyf = params.muy(1) * Fzfmax; Dyr = params.muy(2) * Fzrmax;
prob1.topt = opti.debug.value(T); prob1.tvec = linspace(0,opti.debug.value(T),N+1);

%%%% NaN error debugging 
% opti.debug.g_describe(37)
% opti.debug.x_describe(37)

save('temp_sol_DT_obst_nodyn.mat','prob1');
%% post solution plots
prob1.x = sol.value(xpos); prob1.y = sol.value(ypos);
prob1.vx = sol.value(vx); prob1.vy = sol.value(vy);
prob1.r = sol.value(r); % prob1.Fyr = sol.value(Fyr);
prob1.psi = sol.value(psi);
prob1.delta = sol.value(delta);
prob1.ddelta = sol.value(ddelta);
prob1.Fxf = sol.value(Fxf);
prob1.Fxr = sol.value(Fxr);
prob1.Fxfval = sol.value(Fxfval);
prob1.Fxrval = sol.value(Fxrval);
prob1.Ecy = sol.value(Ecy);

for ii = 1 : (N+1)
    [~, inter(ii),Fxv(ii)] = DT_model(sol.value(X(:,ii)), ...
                            sol.value(U(:,1)), ...
                            params);
    prob1.M_Z(ii) = inter(ii).M_Z;
    prob1.F_Y(ii) = inter(ii).F_Y;
    prob1.F_X(ii) = inter(ii).F_X;
    prob1.vy1(ii) = inter(ii).vy1;
    prob1.vy2(ii) = inter(ii).vy2;
    prob1.vy3(ii) = inter(ii).vy3;
    prob1.vy4(ii) = inter(ii).vy4;
    prob1.vx1(ii) = inter(ii).vx1;
    prob1.vx2(ii) = inter(ii).vx2;
    prob1.vx3(ii) = inter(ii).vx3;
    prob1.vx4(ii) = inter(ii).vx4;
    prob1.alpha1(ii) = inter(ii).alpha1;
    prob1.alpha2(ii) = inter(ii).alpha2;
    prob1.alpha3(ii) = inter(ii).alpha3;
    prob1.alpha4(ii) = inter(ii).alpha4;
    prob1.Fy1(ii) = inter(ii).Fy1;
    prob1.Fy2(ii) = inter(ii).Fy2;
    prob1.Fy3(ii) = inter(ii).Fy3;
    prob1.Fy4(ii) = inter(ii).Fy4;
end

Dxf = params.mux(1) * Fzfmax; Dxr = params.mux(2) * Fzrmax;
Dyf = params.muy(1) * Fzfmax; Dyr = params.muy(2) * Fzrmax;
prob1.topt = sol.value(T); prob1.tvec = linspace(0,sol.value(T),N+1);

save('sol_DT_obst_nodyn.mat','prob1');

%% plotting 

figno = 10;
figure(figno); clf; figno = figno + 1;

plot(prob1.x,prob1.y,'Color','b'); hold on;
fimplicit(@(x,y) obst(x,y) - 1 ,[Xsrt Xend 0 R2+1],'--','Color','k');
ylabel('y [m]','Interpreter','latex');
xlabel('x [m]','Interpreter','latex');

t = prob1.tvec;

for ii = 0:1:round(t(end))
    xi = interp1(t,prob1.x,ii); 
    yi = interp1(t,prob1.y,ii);
    
    yaw = interp1(t,prob1.psi,ii); % yaw at time instant t
    
    [xval, yval] = VehicleShapeNew(xi,yi,yaw,params.lf + params.lr, params.w);

    plot(xval, yval, 'b-','LineWidth',2); % Plotting line segment
end

hold off; xlim([0 inf])

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

figure(figno); clf; figno = figno + 1;

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
plot(prob1.tvec(2:end),prob1.Fxfval); box off;
ylabel('$F_{x(f)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dxf,'LineStyle','--','DisplayName','Fxf (lim)')
yline(-Dxf,'LineStyle','--','DisplayName','Fxf (lim)')

nexttile;
plot(prob1.tvec(2:end),prob1.Fxrval); box off;
ylabel('$F_{x(r)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dxf,'LineStyle','--','DisplayName','Fxf (lim)')
yline(-Dxf,'LineStyle','--','DisplayName','Fxf (lim)')

nexttile;
plot(prob1.tvec,[prob1.Fy1; prob1.Fy2]); box off;
ylabel('$F_{y(f)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dyf,'LineStyle','--','DisplayName','Fyf (lim)')
yline(-Dyf,'LineStyle','--','DisplayName','Fyf (lim)')

nexttile;
plot(prob1.tvec,[prob1.Fy3; prob1.Fy4]); box off;
ylabel('$F_{y(r)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dyr,'LineStyle','--','DisplayName','Fyf (lim)')
yline(-Dyr,'LineStyle','--','DisplayName','Fyf (lim)')

nexttile;
plot(prob1.tvec,rad2deg([prob1.alpha1; prob1.alpha2])); box off;
ylabel('$\alpha_f$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,rad2deg([prob1.alpha3; prob1.alpha4])); box off;
ylabel('$\alpha_f$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,[prob1.vx1; prob1.vx2]*3.6); box off;
ylabel('$v_{x,f}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,[prob1.vx3; prob1.vx4]*3.6); box off;
ylabel('$v_{x,r}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,[prob1.vy1; prob1.vy2]*3.6); box off;
ylabel('$v_{y,f}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

nexttile;
plot(prob1.tvec,[prob1.vy1; prob1.vy2]*3.6); box off;
ylabel('$v_{y,r}$ [km/h]','Interpreter','latex');
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
function [xdot, inter, Fx, Fy] = DT_model(x, u, params)

% ---- inputs ----

% states
vx      = x(3);
vy      = x(4);
r       = x(5);
delta   = x(6);
psi     = x(7);
Fxf     = x(8);
Fxr     = x(9);

% control
ddelta  = u(1);
Fxfval  = u(2);
Fxrval	= u(3);

% ---- parameters ----

% tire 
muxf = params.mux(1); muxr = params.mux(2); 
muyf = params.muy(2); muyr = params.muy(2);
Cxf = params.Cx(1); Cxr = params.Cx(2);
Cyf = params.Cy(1); Cyr = params.Cy(2);
Bxf = params.Bx(1); Bxr = params.Bx(2);
Byf = params.By(1); Byr = params.By(2);
Exf = params.Ex(1); Exr = params.Ex(2);
Eyf = params.Ey(1); Eyr = params.Ey(2);

% vehicle
m   = params.m;
lf  = params.lf; 
lr  = params.lr;
w = params.w;
g   = params.g;
Izz = params.Izz;
Iw  = params.Iw;
Re  = params.Re;
h  = params.h;

% misc
TdFxf = params.TdFxf;
TdFxr = params.TdFxr;

% ---- equations ----
% vehicle lengths and widths
lx1 = lf; lx2 = lf; lx3 = -lr; lx4 = -lr;
ly1 = w/2; ly2 = -w/2; ly3 = w/2; ly4 = -w/2;

% independent wheel velocities 
% longitudanal velocities
vx1 = cos(delta).*(vx - ly1*r) + sin(delta).*(vy + lx1*r);
vx2 = cos(delta).*(vx - ly2*r) + sin(delta).*(vy + lx2*r);
vx3 = vx - ly3*r;
vx4 = vx - ly4*r;
% lateral velocities
vy1 = cos(delta).*(vy + lx1*r) - sin(delta).*(vx - ly1*r);
vy2 = cos(delta).*(vy + lx2*r) - sin(delta).*(vx - ly2*r);
vy3 = vy + lx3*r;
vy4 = vy + lx4*r;

% determining the lateral slips
alpha1 = -atan(vy1./vx1);
alpha2 = -atan(vy2./vx2);
alpha3 = -atan(vy3./vx3);
alpha4 = -atan(vy4./vx4);

% determining the lateral forces on the wheels from Fy
% maximum forces
Fzf = m*g*lr/(lf+lr)/2; 
Fzr = m*g*lf/(lf+lr)/2;
Dxf = muxf.*Fzf; Dxr = muxr.*Fzr; % remove the /2 for a single track model
Dyf = muyf.*Fzf; Dyr = muyr.*Fzr;
% pure slips
Fy01 = Dyf.*sin(Cyf.*atan(Byf.*alpha1-Eyf.*(Byf.*alpha1 - atan(Byf.*alpha1))));
Fy02 = Dyf.*sin(Cyf.*atan(Byf.*alpha2-Eyf.*(Byf.*alpha2 - atan(Byf.*alpha2))));
Fy03 = Dyr.*sin(Cyr.*atan(Byr.*alpha3-Eyr.*(Byr.*alpha3 - atan(Byr.*alpha3))));
Fy04 = Dyr.*sin(Cyr.*atan(Byr.*alpha4-Eyr.*(Byr.*alpha4 - atan(Byr.*alpha4))));
% combined slip
Fy1 = Fy01.*sqrt(1.01 - (Fxf/Dxf).^2);
Fy2 = Fy02.*sqrt(1.01 - (Fxf/Dxf).^2);
Fy3 = Fy03.*sqrt(1.01 - (Fxr/Dxr).^2);
Fy4 = Fy04.*sqrt(1.01 - (Fxr/Dxr).^2);

% the total forces and moments acting on the vehicle
% longitudanal force
F_X1 = Fxf.*cos(delta) - Fy1.*sin(delta);
F_X2 = Fxf.*cos(delta) - Fy2.*sin(delta);
F_X3 = Fxr;
F_X4 = Fxr;
F_X = F_X1 + F_X2 + F_X3 + F_X4;

% lateral force
F_Y1 = Fy1.*cos(delta) + Fxf.*sin(delta);
F_Y2 = Fy2.*cos(delta) + Fxf.*sin(delta);
F_Y3 = Fy3;
F_Y4 = Fy4;
F_Y = F_Y1 + F_Y2 + F_Y3 + F_Y4;

% total moment 
M_Z1 = Fy1.*(lx1*cos(delta) + ly1*sin(delta)) - Fxf.*(ly1*cos(delta) - lx1*sin(delta));
M_Z2 = Fy2.*(lx2*cos(delta) + ly2*sin(delta)) - Fxf.*(ly2*cos(delta) - lx2*sin(delta));
M_Z3 = Fy3*lx3 - Fxr*ly3;
M_Z4 = Fy4*lx4 - Fxr*ly4;
M_Z = M_Z1 + M_Z2 + M_Z3 + M_Z4;

% ---- ODE ----
xdot = [ ... posistion dynamics ...
            vx * cos(psi) - vy * sin(psi); ...    \dot x = vx*cos(psi) - vy*sin(psi) 
            vx * sin(psi) + vy * cos(psi); ...    \dot y = vx*sin(psi) - vy*cos(psi) 
         ... speed dynamics ...
            F_X/m + vy * r;...  \dot vx = FX/m + vy*r
            F_Y/m - vx * r;...  \dot vx = FY/m - vx*r
	        M_Z/Izz;...            \dot r = MZ/Izz
         ... steering angle
         	ddelta;...    \detla = \int ddelta
         ... vehicle orientation (yaw)
         	r;...    \dot \psi = r
         ... limiting the force dynamics with time constants
          	(Fxfval - Fxf)/TdFxf;... 
	    	(Fxrval - Fxr)/TdFxr...
            ]; 
        
% important variables
inter.F_X = F_X;
inter.F_Y = F_Y;
inter.M_Z = M_Z;
inter.Fy1 = Fy1;
inter.Fy2 = Fy2;
inter.Fy3 = Fy3;
inter.Fy4 = Fy4;
inter.alpha1 = alpha1;
inter.alpha2 = alpha2;
inter.alpha3 = alpha3;
inter.alpha4 = alpha4;
inter.vx1 = vx1;
inter.vx2 = vx2;
inter.vx3 = vx3;
inter.vx4 = vx4;
inter.vy1 = vy1;
inter.vy2 = vy2;       
inter.vy3 = vy3;
inter.vy4 = vy4;

Fx = F_X;
Fy = F_Y;

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