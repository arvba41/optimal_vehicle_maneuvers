clc
clear
close all

parameters = struct;
parameters.lf = 1.3;
parameters.lr = 1.5;
parameters.l = parameters.lf + parameters.lr;
parameters.m = 2100;
parameters.Izz = 3900;
parameters.Rw = 0.3;
parameters.Iw = 4;
parameters.g = 9.82;
parameters.mu_xf = 1.2;
parameters.mu_xr = 1.2;
parameters.B_xf = 11.7;
parameters.B_xr = 11.1;
parameters.C_xf = 1.69;
parameters.C_xr = 1.69;
parameters.E_xf = 0.377;
parameters.E_xr = 0.362;
parameters.mu_yf = 0.935;
parameters.mu_yr = 0.961;
parameters.B_yf = 8.86;
parameters.B_yr = 9.3;
parameters.C_yf = 1.19;
parameters.C_yr = 1.19;
parameters.E_yf = -1.21;
parameters.E_yr = -1.11;
parameters.C_xaf = 1.09;
parameters.C_xar = 1.09;
parameters.B_x1f = 12.4;
parameters.B_x1r = 12.4;
parameters.B_x2f = -10.8;
parameters.B_x2r = -10.8;
parameters.C_ykf = 1.08;
parameters.C_ykr = 1.08;
parameters.B_y1f = 6.46;
parameters.B_y1r = 6.46;
parameters.B_y2f = 4.2;
parameters.B_y2r = 4.2;
parameters.C_af = 103600;
parameters.C_ar = 120000;
parameters.N = 30;
parameters.p = 4;
parameters.del_min = -pi/6;
parameters.del_dot_min = -pi/3;
parameters.l_veh = 4;
parameters.w_veh = 1.8;
parameters.Xa = 0;
parameters.Ya = 0;
parameters.R_xi = 2.5;
parameters.R_xo = 7.5;
parameters.R_yi = 50;
parameters.R_yo = 55;
parameters.Xp0 = -5;
parameters.Yp0 = 0;
parameters.phi0 = pi/2;
parameters.vx0 = 30/3.6;
parameters.vy0 = 0;
parameters.r0  = 0;
parameters.wf0 = 0;
parameters.wr0 = 0;
parameters.del0 = 0;
parameters.X_tf = 5;
parameters.Y_tf = 0;


f_ellipse_i = @(x,y) ((x - parameters.Xa)/parameters.R_xi).^parameters.p+ ((y - parameters.Ya)/parameters.R_yi).^parameters.p - 1; % define the ellipse
f_ellipse_o = @(x,y) ((x - parameters.Xa)/parameters.R_xo).^parameters.p+ ((y - parameters.Ya)/parameters.R_yo).^parameters.p - 1; % define the ellipse

theta = 0:2*pi/18:pi; % angles of the vertices
%V = (m*mu*g)*[cos(theta); sin(theta)]; % compute the vertex coordinates
V = [-7.5 0; -7.5 55; 7.5 55; 7.5 0];
P = Polyhedron(V);
parameters.A = P.A;
parameters.b = P.b;
%OptimalControllerMinTime = MinTimeCoGNoFx(parameters);
%OptimalControllerMinTime = MinTimeCoGNoFx(parameters);
%OptimalControllerMinTime = MinTimeCoGNowheelDyn(parameters);
%OptimalControllerMinTime = MinTimeCoGNowheelDyn2Corner(parameters);
OptimalControllerMinTime = MinTimeCoGNowheelDynDT(parameters);
[U_opt, X_opt, tf_opt] = OptimalControllerMinTime.solve( );
Xp = X_opt(1, :);
Yp = X_opt(2, :);

figure(1)
hold on
plot(Xp, Yp, 'b.', 'markersize', 15);
hold on
fimplicit(f_ellipse_i, 'k', 'linewidth', 2);
hold on
fimplicit(f_ellipse_o, 'k', 'linewidth', 2);
hold on
plot(Xp, 0*ones(1, length(Xp)), 'k', 'linewidth', 3);
hold on
plot(Xp, 4*ones(1, length(Xp)), 'k', 'linewidth', 3);
ylim([-0.0, 4.2]);
xlabel('X [m]', 'Interpreter', 'latex');
ylabel('Y [m]', 'Interpreter', 'latex');
set(gca,'FontName','Times New Roman','FontSize',15);
set(gca,'LooseInset',get(gca,'TightInset'));
set(gcf,'unit','centimeters','position',[3 5 20 6.5]);
set(gcf, 'PaperSize', [12 4]);
set(gca, 'ygrid', 'on', 'GridColor', [0.75 0.75 0.75], 'LineWidth', 1);
set(gca, 'xgrid', 'on', 'GridColor', [0.75 0.75 0.75], 'LineWidth', 1);
grid off



