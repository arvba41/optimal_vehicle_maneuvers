clear all; clc;

% load('STmdl_smpl\topt\hrpin_50m_30kmph_init.mat');
load('solved_sol_CV.mat');


%% ---- model parameterization ----

% Pacejka Magic Formula parameters
sel_fric = 3; % selecting tire parameters based on friction
switch sel_fric
    case 1                  
        params.mux  = [1.2      1.2     ];
        params.Bx   = [11.7     11.1    ];
        params.Cx   = [1.69     1.69    ];
        params.Ex   = [0.377    0.362   ];
        params.muy  = [0.935    0.961   ];
        params.By   = [8.86     9.30    ];
        params.Cy   = [1.19     1.19    ];
        params.Ey   = [-1.21    -1.11   ];
    case 2
        params.mux  = [1.06     1.07    ];
        params.Bx   = [12       11.5    ];
        params.Cx   = [1.80     1.80    ];
        params.Ex   = [0.313    0.300   ];
        params.muy  = [0.885    0.911   ];
        params.By   = [10.7     11.3    ];
        params.Cy   = [1.07     1.07    ];
        params.Ey   = [-2.14    -1.97   ];
    case 3
        params.mux  = [0.407    0.409   ];
        params.Bx   = [10.2     9.71    ];
        params.Cx   = [1.96     1.96    ];
        params.Ex   = [0.651    0.624   ];
        params.muy  = [0.383    0.394   ];
        params.By   = [19.1     20      ];
        params.Cy   = [0.550    0.550   ];
        params.Ey   = [-2.10    -1.93   ];
    case 4
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
params.m = 2500;    % Oiginal: 2100
params.lf = 1.3; 
params.lr = 1.5;
params.g = 9.82;
params.Izz = params.m * 3900 / 2500;  % Oiginal: 3900
params.Iw = 4;
params.Re = 0.3;
params.h = 0.50;

% start and final distance
Xsrt    = 0;
Xend    = 75;
Ysrt    = 0;
Yfin    = 76;

% curve parameters
Ri = 72.5; % inner radius
Ro = 77.5; % outer radius
n = 10; % degree of the turn
Xa = 75; % center of the curve 

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;

Vxstart = 70/3.6;

% time constqant for the force 
params.TdFxf = 0.1;
params.TdFxr = 0.1;


%% optimal control problem (OCP)
% ----------------------
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

N = 100;                % number of control intervals

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
% J = -(vx(end) - 0.1*(betax));
% J = (T + 0.1*(betax));
J = (T); 
% J = -(vx(end));
opti.minimize(J);         % minimize time

% ---- dynamic constraints -----
dt = T/N; % length of a control interval
for k=1:N % loop over control intervals 
   
   % Runge-Kutta 4 integration                      
   k1 = ST_model(X(:,k),         U(:,k),    params);
   k2 = ST_model(X(:,k)+dt/2*k1, U(:,k),    params);
   k3 = ST_model(X(:,k)+dt/2*k2, U(:,k),    params);
   k4 = ST_model(X(:,k)+dt*k3,   U(:,k),    params);
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
   
end

% %----------- elipses 
path_in = @(x,y) ((x - Xa)/Ri).^n + (y/Ri).^n;
path_out = @(x,y) ((x - Xa)/Ro).^n + (y/Ro).^n;
opti.subject_to(path_in(xpos,ypos) >= 1);
opti.subject_to(path_out(xpos,ypos) <= 1);

% %---- slack variables ----
% opti.subject_to(Xend - betax <= xpos(N+1) <= Xend + betax)
% opti.subject_to(0 <= betax <= 1);

% ---- initial and boundary conditions --------
opti.subject_to(xpos(1)==Xsrt);        
opti.subject_to(ypos(1)==Ysrt);       

opti.subject_to(xpos(N+1)==Xend); 
opti.subject_to(ypos(N+1)==Yfin);      

opti.subject_to(vx(1)== Vxstart);   
opti.subject_to(vy(1)==0);          

opti.subject_to(psi(1)==pi/2);          
opti.subject_to(psi(N+1)==0);          

opti.subject_to(r(N+1)==0);          

opti.subject_to(delta(N+1)==0);         

opti.subject_to(ddelta(N)==0);         

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive
opti.subject_to(vx>0); % vx not be equal 0

% ---- steering constrains ----
opti.subject_to(-deltamax<=delta<=deltamax);    % steering angle limit
opti.subject_to(-ddeltamax<=ddelta<=ddeltamax); % steering rate limit
% % ---- force constrains ----
Fzfs_max = params.m * params.g * params.lr / (params.lf + params.lr);
Fzrs_max = params.m * params.g * params.lf / (params.lf + params.lr);
Dxf = params.mux(1) * Fzfs_max; 
Dxr = params.mux(2) * Fzrs_max;
opti.subject_to(-Dxf <= Fxfval <=Dxf);    
opti.subject_to(-Dxr <= Fxrval <=0);      % rear wheel braking only
% opti.subject_to(Fxrval==0);      % rear wheel no force
opti.subject_to(-Dxf <= Fxf <= Dxf);    
opti.subject_to(-Dxr <= Fxr <= 0);      % rear wheel braking only
% opti.subject_to(Fxr==0);      % rear wheel braking no force

% ---- initial values for solver ---
% define all the initial values here 

% % % % % opti.set_initial(xpos, prob1.x);
% % % % % opti.set_initial(ypos, prob1.y);
% % % % % 
% % % % % opti.set_initial(vx, prob1.vx);
% % % % % opti.set_initial(vy, prob1.vy);
% % % % % 
% % % % % opti.set_initial(r, prob1.r);
% % % % % 
% % % % % opti.set_initial(delta, prob1.delta);
% % % % % opti.set_initial(ddelta, prob1.ddelta);
% % % % % 
% % % % % opti.set_initial(psi, prob1.psi);
% % % % % 
% % % % % opti.set_initial(T, prob1.topt);
% % % % % 
% % % % % opti.set_initial(Fxf, prob1.Fxf);
% % % % % opti.set_initial(Fxr, prob1.Fxr);
% % % % % 
% % % % % opti.set_initial(Fxfval, prob1.Fxfval);
% % % % % opti.set_initial(Fxrval, prob1.Fxrval);

% % % % % % ---- initial values for solver ---
% opti.set_initial(x, Xsrt);
% opti.set_initial(y, Ysrt);

opti.set_initial(vx, Vxstart);
opti.set_initial(vy, 0);

opti.set_initial(r, 0);

opti.set_initial(delta, 0);
opti.set_initial(ddelta, 0);

opti.set_initial(psi, 0);

opti.set_initial(T, 2.4);

opti.set_initial(Fxfval, 100);
% opti.set_initial(Fxrval, 0);

opti.set_initial(Fxf, 100);
% opti.set_initial(Fxr, 0);


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
                                        
for ii = 1 : (N+1)
    [~, inter(ii),Fxv(ii)] = ST_model(opti.debug.value(X(:,ii)), ...
                            opti.debug.value(U(:,1)), ...
                            params);
    prob1.M_Z(ii) = inter(ii).M_Z;
    prob1.F_Y(ii) = inter(ii).F_Y;
    prob1.F_X(ii) = inter(ii).F_X;
    prob1.vyr(ii) = inter(ii).vyr;
    prob1.vyf(ii) = inter(ii).vyf;
    prob1.vxr(ii) = inter(ii).vxr;
    prob1.vxf(ii) = inter(ii).vxf;
    prob1.alphar(ii) = inter(ii).alphar;
    prob1.alphaf(ii) = inter(ii).alphaf;
    prob1.Fyr(ii) = inter(ii).Fyr;
    prob1.Fyf(ii) = inter(ii).Fyf;
end


Dxf = params.mux(1) * Fzfs_max; 
Dxr = params.mux(2) * Fzrs_max;
Dyf = params.muy(1) * Fzfs_max;
Dyr = params.muy(2) * Fzrs_max;

prob1.topt = opti.debug.value(T); prob1.tvec = linspace(0,opti.debug.value(T),N+1);
%%%% NaN error debugging 


save('temp_sol_CV.mat','prob1')
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
                                        
for ii = 1 : (N+1)
    [~, inter(ii),Fxv(ii)] = ST_model(sol.value(X(:,ii)), ...
                            sol.value(U(:,1)), ...
                            params);
    prob1.M_Z(ii) = inter(ii).M_Z;
    prob1.F_Y(ii) = inter(ii).F_Y;
    prob1.F_X(ii) = inter(ii).F_X;
    prob1.vyr(ii) = inter(ii).vyr;
    prob1.vyf(ii) = inter(ii).vyf;
    prob1.vxr(ii) = inter(ii).vxr;
    prob1.vxf(ii) = inter(ii).vxf;
    prob1.alphar(ii) = inter(ii).alphar;
    prob1.alphaf(ii) = inter(ii).alphaf;
    prob1.Fyr(ii) = inter(ii).Fyr;
    prob1.Fyf(ii) = inter(ii).Fyf;
end

Dxf = params.mux(1) * Fzfs_max; 
Dxr = params.mux(2) * Fzrs_max;
Dyf = params.muy(1) * Fzfs_max;
Dyr = params.muy(2) * Fzrs_max;

prob1.topt = sol.value(T); prob1.tvec = linspace(0,sol.value(T),N+1);

save('solved_sol_CV.mat','prob1');

%% plotting 

figno = 10;
figure(figno); clf; figno = figno + 1;

plot(prob1.x,prob1.y,'Color','b'); hold on; grid off;
fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k');
fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k')
ylabel('$y$ [m]','Interpreter','latex');
xlabel('$x$ [m]','Interpreter','latex');
grid on;

t = prob1.tvec;

for ii = 0:1:round(t(end))
    xi = interp1(t,prob1.x,ii); 
    yi = interp1(t,prob1.y,ii);
    
    yaw = interp1(t,prob1.psi,ii); % yaw at time instant t
    
    [xval, yval] = VehicleShapeNew(xi,yi,yaw,params.lf+params.lr,0);

    plot(xval, yval, 'b-','LineWidth',4); % Plotting line segment
end

hold off; % xlim([0 inf])

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
xlabel('$t$ [s]','Interpreter','latex'); hold on;
plot(prob1.tvec,Dxf,'k','LineStyle','--','DisplayName','Fxf (lim)')
plot(prob1.tvec,-Dxf,'k','LineStyle','--','DisplayName','Fxf (lim)')

nexttile;
plot(prob1.tvec,prob1.Fxr); box off;
ylabel('$F_{x(r)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex'); hold on;
plot(prob1.tvec,Dxr,'k','LineStyle','--','DisplayName','Fxf (lim)')
plot(prob1.tvec,-Dxr,'k','LineStyle','--','DisplayName','Fxf (lim)')

nexttile;
plot(prob1.tvec,prob1.Fyf); box off;
ylabel('$F_{y(f)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex'); hold on;
plot(prob1.tvec,Dyf,'k','LineStyle','--','DisplayName','Fyf (lim)')
plot(prob1.tvec,-Dyf,'k','LineStyle','--','DisplayName','Fyf (lim)')

nexttile;
plot(prob1.tvec,prob1.Fyr); box off;
ylabel('$F_{y(r)}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex'); hold on;
plot(prob1.tvec,Dyr,'k','LineStyle','--','DisplayName','Fyf (lim)')
plot(prob1.tvec,-Dyr,'k','LineStyle','--','DisplayName','Fyf (lim)')

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
hold on;
plot(prob1.tvec,Fxv,'--','Color','k');

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
                                                    
function [xdot, inter, Fx] = ST_model(x ,u  ,params)

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
Fxrval  = u(3);

% ---- parameters ----

% magic tire formula 
muxf = params.mux(1); muxr = params.mux(2); 
muyf = params.muy(2); muyr = params.muy(2);
Cxf = params.Cx(1); Cxr = params.Cx(2);
Cyf = params.Cy(1); Cyr = params.Cy(2);
Bxf = params.Bx(1); Bxr = params.Bx(2);
Byf = params.By(1); Byr = params.By(2);
Exf = params.Ex(1); Exr = params.Ex(2);
Eyf = params.Ey(1); Eyr = params.Ey(2);

% ST model
% ST model parameters
m   = params.m;
lf  = params.lf; 
lr  = params.lr;
g   = params.g;
Izz = params.Izz;
Iw  = params.Iw;
Re  = params.Re;
h   = params.h;

% misc parameters
TdFxf = params.TdFxf; TdFxr = params.TdFxr; 

% ---- independent wheel velocities ----
vxf = vx*cos(delta) + sin(delta)*(vy + lf*r); 
vyf = cos(delta)*(vy + lf*r) - vx*sin(delta);
vxr = vx;
vyr = vy + lr*r;
% ---- determining the lateral slips ----
alphaf = -atan(vyf./vxf);
alphar = -atan(vyr./vxr);
% ---- determining the lateral forces on the wheels from Fy ----
% nomianl forces
Fzf = m*g*lr/(lf+lr); 
Fzr = m*g*lf/(lf+lr);
Dxf = muxf.*Fzf; Dxr = muxr.*Fzr;
Dyf = muyf.*Fzf; Dyr = muyr.*Fzr;
% pure splip
Fy0f = Dyf.*sin(Cyf.*atan(Byf.*alphaf-Eyf.*(Byf.*alphaf - atan(Byf.*alphaf))));
Fy0r = Dyr.*sin(Cyr.*atan(Byr.*alphar-Eyr.*(Byr.*alphar - atan(Byr.*alphar))));
% combined slip
% Fyf = Fy0f.*sqrt(1 - (min(max(Fxf,-Dxf),Dxf)/Dxf).^2);
% Fyr = Fy0r.*sqrt(1 - (min(max(Fxr,-Dxr),Dxr)/Dxr).^2);
Fyf = Fy0f.*sqrt(1.01 - (Fxf/Dxf)^2);
Fyr = Fy0r.*sqrt(1.01 - (Fxr/Dxr)^2);
% Fyf = Fy0f.*sqrt(1 - (Fxf/Dxf)^2);
% Fyr = Fy0r.*sqrt(1 - (Fxr/Dxr)^2);
% ---- the total forces and moments acting on the vehcile ----
F_X = cos(delta).*Fxf ...
        - sin(delta).*Fyf ...
        + Fxr;  
F_Y = cos(delta).*Fyf ...
        + sin(delta).*Fxf ...
        + Fyr;
M_Z = lf*cos(delta).*Fyf ...
        + lf*sin(delta).*Fxf ...
        - lr*Fyr;
    
% ---- outputs ----

% update states
xdot = [ ... posistion dynamics ...
            vx*cos(psi) - x(4)*sin(psi); ...    \dot x = vx*cos(psi) - vy*sin(psi) 
            vx*sin(psi) + x(4)*cos(psi); ...    \dot y = vx*sin(psi) - vy*cos(psi) 
         ... speed dynamics ...
            F_X/m + x(4)*x(5);...  \dot vx = FX/m + vy*r
            F_Y/m - x(3)*x(5);...  \dot vx = FY/m - vx*r
            M_Z/Izz;...            \dot r = MZ/Izz
         ... steering angle
            ddelta;...  \detla = \int ddelta
         ... vehicle orientation (yaw)
            r;...   \dot \psi = r
         ... limiting the force dynamics with time constants
            (Fxfval - Fxf)/TdFxf;... 
            (Fxrval - Fxr)/TdFxr...
        ];

% important variables
inter.F_X = F_X;
inter.F_Y = F_Y;
inter.M_Z = M_Z;
inter.Fyf = Fyf;
inter.Fyr = Fyr;
inter.alphaf = alphaf;
inter.alphar = alphar;
inter.vxf = vxf;
inter.vxr = vxr;
inter.vyf = vyf;
inter.vyr = vyr;

% important variables
Fx = F_X;

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