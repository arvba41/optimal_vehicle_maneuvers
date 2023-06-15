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

params.Bx1 = [12.4 12.4];
params.Bx2 = [-10.8 -10.8];
params.Cxa = [1.09 1.09];
params.By1 = [6.46 6.46];
params.By2 = [4.2 4.2];
params.Cyk = [1.08 1.08];

% vehicle parameters
params.m = 2100;
params.lf = 1.3; 
params.lr = 1.5;
params.g = 9.82;
params.Izz = 3900;
params.Iw = 4;
params.Re = 0.3;

% start and final distance
Xsrt    = -10;
Xend    = 10;
Ysrt   = 0;
Yend    = 0;

% hairpin parameters
R1i = 2;
R2i = 1;
R1o = R1i + 5;
R2o = R2i + 5;

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;

% start of the hairpin turn 
Vxstart = 30/3.6;

Fxfmax = params.mux(1) * ( params.m * params.g * params.lr ) / (params.lf + params.lr);
Fxrmax = params.mux(2) * ( params.m * params.g * params.lf ) / (params.lf + params.lr);

Tfmax = Fxfmax * params.Re;
Trmax = Fxrmax * params.Re;

dTmax = 1e3;

%% yop
% % yops Times: t t0 tf 
% % yops States: xpos ypos psi vx vy r delta omegaf omegar Tf Tr
% % yops Control: ddelta dTf dTr 
% % 
% % % ---- inputs ----
% % % sates
% % X = [xpos; ypos; psi; vx; vy; r; delta; omegaf; omegar; Tf; Tr];
% % % control
% % U = [ddelta; dTf; dTr];
% % 
% % % ---- OCP ----
% % ocp = yop.ocp('ST hairpin Problem');
% % 
% % % constraints
% % ocp.st( vx > 0 );
% % ocp.st( t >= 0 );
% % % ---- steering constrains ----
% % ocp.st( -deltamax <= delta <= deltamax );
% % ocp.st( -ddeltamax <= ddelta <= ddeltamax );
% % % ---- torque constrains ----
% % ocp.st( -Tfmax <= Tf <= Tfmax);    
% % ocp.st( -Trmax <= Tr <= 0); 
% % % ---- torque rate constrains ----
% % ocp.st( -dTmax <= dTf <= dTmax);    
% % ocp.st( -dTmax <= dTr <= dTmax); 
% % 
% % ocp.st( ocp.st( der(X) == ST_model(X, U, params) ) );
% % 
% % % initial conditions
% % ocp.st(0 == t0 <= tf); 
% % 
% % ocp.st( xpos(t0) == Xsrt );
% % ocp.st( ypos(t0) == Ysrt );
% % ocp.st( xpos(tf) == Xend );
% % ocp.st( ypos(tf) == Yend );
% % 
% % ocp.st( psi(t0) == 0 );
% % 
% % ocp.st( vx(t0) == Vxstart );
% % ocp.st( vy(t0) == 0 );
% % 
% % ocp.st( r(t0) == 0 );
% % 
% % ocp.st( omegaf(t0) == Vxstart / params.Re );
% % ocp.st( omegar(t0) == Vxstart / params.Re );
% % 
% % ocp.st( Tf(t0) == 0 );
% % ocp.st( Tr(t0) == 0 );
% % 
% % % ---- onjective ----
% % ocp.min(tf);
% % 
% % % ---- solving OCP ----
% % sol = ocp.solve;
% % 
% % %% Yop plots 
% % 
% % figure(1); clf; 
% % 
% % tiledlayout('flow');
% % 
% % nexttile([1 2])
% % sol.plot(xpos,ypos); grid on;
% % xlabel('x [m]'); ylabel('y [m]')
% % 
% % nexttile;
% % sol.plot(t,vx*3.6); grid on;
% % xlabel('t [s]'); ylabel('v_x [km/h]')
% % 
% % nexttile;
% % sol.plot(t,vy*3.6); grid on;
% % xlabel('t [s]'); ylabel('v_y [km/h]')
% % 
% % nexttile;
% % sol.plot(t,r); grid on;
% % xlabel('t [s]'); ylabel('r [rad/s]')
% % 
% % nexttile;
% % sol.plot(t,omegaf*30/pi); grid on;
% % xlabel('t [s]'); ylabel('\omega_f [rpm]')
% % 
% % nexttile;
% % sol.plot(t,omegar*30/pi); grid on;
% % xlabel('t [s]'); ylabel('\omega_r [rpm]')
% % 
% % nexttile;
% % sol.plot(t,Tf); grid on;
% % xlabel('t [s]'); ylabel('T_f [Nm]')
% % 
% % nexttile;
% % sol.plot(t,Tr); grid on;
% % xlabel('t [s]'); ylabel('T_r [Nm]')


%% optimal control problem 
% ----------------------
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP

N = 60;                     % number of control intervals

opti = casadi.Opti();       % Optimization problem

% ---- decision variables ---------
X = opti.variable(11,N+1);   % state trajectory
xpos    = X(1,:);
ypos    = X(2,:);
psi     = X(3,:);
vx      = X(4,:);
vy      = X(5,:);
r       = X(6,:);
delta   = X(7,:);
omegaf  = X(8,:);
omegar  = X(9,:);
Tf      = X(10,:);
Tr      = X(11,:);
U = opti.variable(3,N);     % control trajectory (steering rate and front wheel force)
ddelta  = U(1,:);
dTf     = U(2,:);
dTr     = U(3,:);
T = opti.variable();        % final time

% ---- objective ----
opti.minimize(T);           % minimize time

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

% ---- hairpin ----
% path_in = @(x,y) ((x-Xa)/R1).^pow + (y/R2).^pow;
% path_out = @(x,y) ((x-Xa)/(R1+Owdt)).^pow + (y/(R2+Owdty)).^pow;
% opti.subject_to(path_in(xpos,ypos) >= 1);
% opti.subject_to(path_out(xpos,ypos) <= 1);

% ---- boundary conditions --------
opti.subject_to(xpos(1) == Xsrt);        
opti.subject_to(ypos(1) == Ystrt);
opti.subject_to(xpos(N+1) == Xend); 
opti.subject_to(ypos(N+1) == Yfin);     
opti.subject_to(psi(1) == 0);  
opti.subject_to(psi(N+1) == 0);
opti.subject_to(vx(1) == Vxstart);   
opti.subject_to(vy(1) == 0);
% opti.subject_to(vy(N+1)==0);      
% opti.subject_to(r(1)==0);         
% opti.subject_to(r(N+1)==0);  
% opti.subject_to(delta(1)==0);           
% opti.subject_to(ddelta(1)==0); 
% opti.subject_to(delta(N+1)==0);         
% opti.subject_to(ddelta(N+1)==0); 
opti.subject_to(omegaf(1) == Vxstart / params.Re);   
opti.subject_to(omegar(1) == Vxstart / params.Re); 
opti.subject_to(Tf(1) == 0);
opti.subject_to(Tr(1) == 0);
% opti.subject_to(dTf(1)   == 0);
% opti.subject_to(dTr(1)   == 0);

% ---- misc. constraints  ----------
opti.subject_to( T >= 0 ); % Time must be positive
opti.subject_to( vx > 0 ); % vx not be equal 0

% ---- steering constrains ----
opti.subject_to( -deltamax <= delta <= deltamax);    % steering angle limit
opti.subject_to( -ddeltamax <= ddelta <= ddeltamax); % steering rate limit

% ---- force constrains ----
% opti.subject_to( -Fxfmax <= Fxf <= Fxfmax);    
% opti.subject_to( -Fxrmax <= Fxr <= 0); 

% ---- torque constrains ----
opti.subject_to( -Tfmax <= Tf <= Tfmax);    
opti.subject_to( -Trmax <= Tr <= 0); 

% ---- torque rate constrains ----
opti.subject_to( -dTmax <= dTf <= dTmax);    
opti.subject_to( -dTmax <= dTr <= dTmax); 

% ---- initial values for solver ---
opti.set_initial(vx, Vxstart ); % vx not be equal 0
opti.set_initial(T, 0.5);
opti.set_initial(dTf, 0);
opti.set_initial(dTr, 0);
opti.set_initial(ddelta, 0);

% ---- maximum iterations ----
p_opts = struct('expand',true);
s_opts = struct('max_iter',10000);

% ---- solve NLP              ------
opti.solver('ipopt',p_opts,s_opts); % set numerical backend
sol = opti.solve();   % actual solve

%% debuggin plots
prob.xpos = opti.debug.value(xpos);
prob.ypos = opti.debug.value(ypos);
prob.psi = opti.debug.value(psi);
prob.vx = opti.debug.value(vx);
prob.vy = opti.debug.value(vy);
prob.r = opti.debug.value(r);
prob.delta = opti.debug.value(delta);
prob.omegaf = opti.debug.value(omegaf);
prob.omegar = opti.debug.value(omegar);
prob.Tf = opti.debug.value(Tf);
prob.Tr = opti.debug.value(Tr);


%% test odes

val = [0; 0; 0; Vxstart; 0; 0; 0; Vxstart/params.Re; Vxstart/params.Re; 0; 0];
inp = [0; -100; -100];
% [xdot] = ST_model(val,inp,params)

X(:,1) = val

%% interation
dt = 5/5;
for k=1:5 % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = ST_model(X(:,k),         inp,   params);
   k2 = ST_model(X(:,k)+dt/2*k1, inp,   params);
   k3 = ST_model(X(:,k)+dt/2*k2, inp,   params);
   k4 = ST_model(X(:,k)+dt*k3,   inp,   params);
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   X(:,k+1) = x_next
end

%% functions 

function [xdot] = ST_model(x,u,params)
% ----- inputs -----

% states
xpos    = x(1);
ypos    = x(2);
psi     = x(3);
vx      = x(4);
vy      = x(5);
r       = x(6);
delta   = x(7);
omegaf  = x(8);
omegar  = x(9);
Tf      = x(10);
Tr      = x(11);

% inputs
ddelta  = u(1,:);
dTf     = u(2,:);
dTr     = u(3,:);


% ST model parameters
m   = params.m;
lf  = params.lf; 
lr  = params.lr;
g   = params.g;
Izz = params.Izz;
Iw  = params.Iw;
Re  = params.Re;

% tire parameters
muxf = params.mux(1); muxr = params.mux(2); 
muyf = params.muy(2); muyr = params.muy(2);
Cxf = params.Cx(1); Cxr = params.Cx(2);
Cyf = params.Cy(1); Cyr = params.Cy(2);
Bxf = params.Bx(1); Bxr = params.Bx(2);
Byf = params.By(1); Byr = params.By(2);
Exf = params.Ex(1); Exr = params.Ex(2);
Eyf = params.Ey(1); Eyr = params.Ey(2);

Bx1f = params.Bx1(1); Bx1r = params.Bx1(2);
Bx2f = params.Bx2(1); Bx2r = params.Bx2(2);
Cxaf = params.Cxa(1); Cxar = params.Cxa(2);
By1f = params.By1(1); By1r = params.By1(2);
By2f = params.By2(1); By2r = params.By2(2);
Cykf = params.Cyk(1); Cykr = params.Cyk(2);

% ----- ST equations -----

% wheel velocities
temp = [cos(delta) sin(delta); -sin(delta) cos(delta)] * ( [vx; vy] + r * [0; lf]);
vxf = temp(1);
vyf = temp(2);
temp = [1 0; 0 1] *( [vx; vy] + r * [0; lr]);
vxr = temp(1);
vyr = temp(2);
% -------- alternative
% vxf = vx * cos(delta) + (vy + lf * r) * sin(delta);
% vxr = vx;

% longitudinal tire slip 
kappaf = (Re * omegaf - vxf) / vxf;
kappar = (Re * omegar - vxr) / vxr;

% lateral tire slips;
alphaf = -atan(vyf / vxf);
alphar = -atan(vyr / vxr);
% alphaf = delta - atan( ( vy + lf * r) / vx );
% alphar = - atan( ( vy - lr * r) / vx );

% The nominal normal force resting on wheel in steady state 
% Fzf = m * g * lr / (lr + lf);
% Fzr = m * g * lf / (lr + lf);

% longitudinal nominal tire forces 
% Fx0f = muxf * Fzf * sin( Cxf * atan(Bxf * kappaf - Exf * (Bxf * kappaf - atan( Bxf * kappaf ) ) ) );
% Fx0r = muxr * Fzr * sin( Cxr * atan(Bxr * kappar - Exr * (Bxr * kappar - atan( Bxr * kappar ) ) ) );

% lateral nominal tire forces 
% Fy0f = muyf * Fzf * sin( Cyf * atan(Byf * alphaf - Eyf * (Bxf * alphaf - atan( Bxf * alphaf ) ) ) );
% Fy0r = muyr * Fzr * sin( Cyr * atan(Byr * alphar - Eyr * (Byr * alphar - atan( Byr * alphar ) ) ) );

% weighting functions longitudinal 
% Hxaf = Bx1f * cos ( atan( Bx2f * kappaf ) );
% Gxaf = cos( Cxaf * atan( Hxaf * alphaf) );
% Fxf = Fx0f * Gxaf;

Hxar = Bx1r * cos ( atan( Bx2r * kappar ) );
Gxar = cos( Cxar * atan( Hxar * alphar) );
Fxr = Fx0r * Gxar;

% weighting functions lateral
Hykf = By1f * cos ( atan( By2f * alphaf ) );
Gyaf = cos( Cykf * atan( Hykf * kappaf ) );
Fyf = Fy0f * Gyaf;

Hykr = By1r * cos ( atan( By2r * alphar ) );
Gyar = cos( Cykr * atan( Hykr * kappar ) );
Fyr = Fy0r * Gyar;

% The total forces and moment acting on the vehicle center of mass 
Fx = cos(delta) * Fxf - sin(delta) * Fyf + Fxr;
Fy = cos(delta) * Fyf + sin(delta) * Fxf + Fyr;
Mz = lf * cos(delta) * Fyf  + lf * sin(delta) * Fxf - lr * Fyr;

% ----- ODEs ------
xposdot     = vx * cos(psi) - vy * sin(psi);
yposdot     = vx * sin(psi) + vy * cos(psi);
psidot      = r;
vxdot       = Fx / m + vy * r;
vydot       = Fy / m - vx * r;
rdot        = Mz / Izz;
deltadot    = ddelta;
% omegafdot   = ( Tf - Re * Fxf ) / Iw;
% omegardot   = ( Tr - Re * Fxr ) / Iw;
% Tfdot       = dTf;
% Trdot       = dTr;

% ----- ouputs -----
xdot     = [xposdot; ...
            yposdot; ...
            psidot; ...
            vxdot; ...
            vydot; ...
            rdot; ...
            deltadot; ...
%             omegafdot; ...
%             omegardot; ...
%             Tfdot; ...
%             Trdot ...
            ];
end



