clear all; clc; % close all;

%% Model parameterization 
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
params.Izz = 3900; % 25% increase in inertia (case 1 with 50m and 50kph) %% 10% increase in inertia 
params.Iw = 4;
params.Re = 0.3;

% start and final distance
Xsrt    = 5;
Xfin    = 20/2;
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

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;
% Ymax = 3; % upper limit 

Vxstart = 30/3.6;

% time constqant for the force 
TdFxf = 0.2;
TdFxr = 0.2;

%% equations for the forces and moment

% ST model parameters
m   = params.m;
lf  = params.lf; 
lr  = params.lr;
g   = params.g;
Izz = params.Izz;
Iw  = params.Iw;
Re  = params.Re;

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
vxf = @(vx,vy,r,delta,Fxf,Fxr) vx.*cos(delta) + sin(delta).*(vy + lf.*r); 
vyf = @(vx,vy,r,delta,Fxf,Fxr) cos(delta).*(vy + lf.*r) - vx.*sin(delta);
vxr = @(vx,vy,r,delta,Fxf,Fxr) vx;
vyr = @(vx,vy,r,delta,Fxf,Fxr) vy + lr.*r;
% ---- determining the lateral slips ----
alphaf = @(vx,vy,r,delta,Fxf,Fxr) -atan(vyf(vx,vy,r,delta,Fxf,Fxr)./vxf(vx,vy,r,delta,Fxf,Fxr));
alphar = @(vx,vy,r,delta,Fxf,Fxr) -atan(vyr(vx,vy,r,delta,Fxf,Fxr)./vxr(vx,vy,r,delta,Fxf,Fxr));
% ---- determining the lateral forces on the wheels from Fy ----
% maximum forces
Fzf = m*g*lr/(lf+lr); 
Fzr = m*g*lf/(lf+lr);
Dxf = muxf.*Fzf; Dxr = muxr.*Fzr;
Dyf = muyf.*Fzf; Dyr = muyr.*Fzr;
Fy0f = @(vx,vy,r,delta,Fxf,Fxr) Dyf.*sin(Cyf.*atan(Byf.*alphaf(vx,vy,r,delta,Fxf,Fxr)-Eyf.*(Byf.*alphaf(vx,vy,r,delta,Fxf,Fxr) - atan(Byf.*alphaf(vx,vy,r,delta,Fxf,Fxr)))));
Fy0r = @(vx,vy,r,delta,Fxf,Fxr) Dyr.*sin(Cyr.*atan(Byr.*alphar(vx,vy,r,delta,Fxf,Fxr)-Eyr.*(Byr.*alphar(vx,vy,r,delta,Fxf,Fxr) - atan(Byr.*alphar(vx,vy,r,delta,Fxf,Fxr)))));
Fyf = @(vx,vy,r,delta,Fxf,Fxr) Fy0f(vx,vy,r,delta,Fxf,Fxr).*sqrt(1 - (Fxf/Dxf).^2);
Fyr = @(vx,vy,r,delta,Fxf,Fxr) Fy0r(vx,vy,r,delta,Fxf,Fxr).*sqrt(1 - (Fxr/Dxr).^2);
% ---- the total forces and moments acting on the vehcile ----
F_X = @(vx,vy,r,delta,Fxf,Fxr)  cos(delta).*Fxf ...
                        - sin(delta).*Fyf(vx,vy,r,delta,Fxf,Fxr) ...
                        + Fxr;  
F_Y = @(vx,vy,r,delta,Fxf,Fxr)  cos(delta).*Fyf(vx,vy,r,delta,Fxf,Fxr) ...
                        + sin(delta).*Fxf ...
                        + Fyr(vx,vy,r,delta,Fxf,Fxr);
M_Z = @(vx,vy,r,delta,Fxf,Fxr)  lf*cos(delta).*Fyf(vx,vy,r,delta,Fxf,Fxr) ...
                        + lf*sin(delta).*Fxf ...
                        - lr*Fyr(vx,vy,r,delta,Fxf,Fxr);

%---- obstacle ----
path_in = @(x,y) ((x-Xa)/R1).^pow + (y/R2).^pow;
path_out = @(x,y) ((x-Xa)/(R1+Owdt)).^pow + (y/(R2+Owdty)).^pow;

                
%% plottings

figno_c = 10;
% clearing plots
figure(figno_c); clf;
figure(figno_c+1); clf;
figure(figno_c+2); clf;
figure(figno_c+3); clf;

for ii = 7:8 % 2:2:4 % 1:2:3 % 
    switch ii
        case 1 
            load("hrp_50m_30kmph_t_opt_init_res_v2.mat");
            lgnd(ii) = "min $t$, nom $I_{zz}$";
            clr = [0 0.4470 0.7410];
        case 2
            load("hrp_50m_30kmph_intertial_inc_10pct_t_opt_init_res.mat");
            lgnd(ii) = "min $t$, +10\% $I_{zz}$";
            clr = [0 0.4470 0.7410];
        case 3
            load("hrp_50m_30kmph_vf_opt_init_res_v2.mat");
            lgnd(ii) = "min $-v_f$, $I_{zz}$";
            clr = [0.8500 0.3250 0.0980];
        case 4
            load("hrp_50m_30kmph_intertial_inc_10pct_vf_opt_init_res.mat");
            lgnd(ii) = "min $-v_f$, +10\% $I_{zz}$";
            clr = [0.8500 0.3250 0.0980];
        case 5
            load("hrp_50m_30kmph_t_opt_free_xpos.mat");
            lgnd(ii) = "min $-v_f$, free xpos";
            clr = [0 0.4470 0.7410];
        case 6
            load("hrp_50m_30kmph_vf_opt_free_xpos.mat");
            lgnd(ii) = "min $-v_f$, free xpos";
            clr = [0.8500 0.3250 0.0980];    
        case 7
            load("hrpo6_55m_30kmph_t_opt_free_vx_xpos.mat");
            lgnd(ii) = "min $-v_f$, free xpos";
            clr = [0 0.4470 0.7410];
            pow = 6;
            path_in = @(x,y) ((x-Xa)/R1).^pow + (y/R2).^pow;
            path_out = @(x,y) ((x-Xa)/(R1+Owdt)).^pow + (y/(R2+Owdty)).^pow;
        case 8
            load("hrpo6_55m_30kmph_vf_sub_opt_free_vx_xpos.mat");
            lgnd(ii) = "min $-v_f$, free xpos";
            clr = [0.8500 0.3250 0.0980];    
            pow = 6;
            path_in = @(x,y) ((x-Xa)/R1).^pow + (y/R2).^pow;
            path_out = @(x,y) ((x-Xa)/(R1+Owdt)).^pow + (y/(R2+Owdty)).^pow;
    end
    
    %% plots
    figno = figno_c;
    figure(figno); figno = figno + 1;

    p(ii) = plot(prob1.x,prob1.y,'DisplayName',char(lgnd(ii))); hold on;
    p(ii).Color = clr;
    if ii == 3 || ii == 4 || ii == 6 || ii == 8
        p1 = fimplicit(@(x,y) path_in(x,y) - 1 ,[0 2*Xfin 0 Ohgt+Owdty],'Color','k','LineStyle','--','LineWidth',2);
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
        p1 = fimplicit(@(x,y) path_out(x,y) - 1 ,[0 2*Xfin 0 Ohgt+Owdty],'Color','k','LineStyle','--','LineWidth',2);
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
        legend('Interpreter','latex');
    end
    set(gca,'YGrid','on','GridLineStyle','--','GridColor','k');  
    ylabel('y [m]','Interpreter','latex');
    xlabel('x [m]','Interpreter','latex');
    
    t = prob1.tvec;

    for jj = 0:1:round(t(end))
        xi = interp1(t,prob1.x,jj); 
        yi = interp1(t,prob1.y,jj);

        yaw = interp1(t,prob1.psi,jj); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,lf+lr,0);

        p1 = plot(xval, yval,'LineWidth',2); % Plotting line segment
        p1.Color = p(ii).Color;
        
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
    
    h = findall(gcf,'Type','axes'); % An array if you have subplots
    set(h, 'TickLabelInterpreter', 'latex')

    figure(figno); figno = figno + 1;
    
    a(1) = subplot(5,2,1);
    plot(prob1.tvec,prob1.vx*3.6); box off; hold on;
    ylabel('$v_x$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');

    a(2) = subplot(5,2,2);
    plot(prob1.tvec,prob1.vy*3.6); box off; hold on;
    ylabel('$v_y$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');

    a(3) = subplot(5,2,3);
    plot(prob1.tvec,rad2deg(prob1.r)); box off; hold on;
    ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');

    a(4) = subplot(5,2,4);
    plot(prob1.tvec,rad2deg(wrapToPi(prob1.psi))); box off; hold on;
    ylabel('$\psi$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');

    a(5) = subplot(5,2,5);
    plot(prob1.tvec,rad2deg(prob1.delta)); box off; hold on;
    ylabel('$\delta$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    yline(rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
    yline(-rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
    
    a(6) = subplot(5,2,6);
    plot(prob1.tvec(2:end),rad2deg(prob1.ddelta)); box off; hold on;
    ylabel('$\dot \delta$ [deg/s]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    yline(rad2deg(ddeltamax),'LineStyle','--','DisplayName','$\dot \delta$ (lim)')
    yline(-rad2deg(ddeltamax),'LineStyle','--','DisplayName','$\dot \delta$ (lim)')

    a(7) = subplot(5,2,7);
    plot(prob1.tvec,prob1.vxf*3.6); box off; hold on;
    ylabel('$v_{x(f)}$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');

    a(8) = subplot(5,2,8);
    plot(prob1.tvec,prob1.vxr*3.6); box off; hold on;
    ylabel('$v_{x(r)}$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    a(9) = subplot(5,2,9);
    plot(prob1.tvec,prob1.vyf*3.6); box off; hold on;
    ylabel('$v_{y(f)}$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');

    a(10) = subplot(5,2,10);
    plot(prob1.tvec,prob1.vyf*3.6); box off; hold on;
    ylabel('$v_{y(f)}$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');

    h = findall(gcf,'Type','axes'); % An array if you have subplots
    set(h, 'TickLabelInterpreter', 'latex')

    linkaxes(a,'x'); clear a

    figure(figno); figno = figno + 1;

    a(1) = subplot(3,2,1);
    plot(prob1.tvec,prob1.Fxf); box off; hold on;
    ylabel('$F_{x(f)}$ [N]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    yline(Dxf,'LineStyle','--','DisplayName','Fxf (lim)');
    yline(-Dxf,'LineStyle','--','DisplayName','Fxf (lim)');

    a(2) = subplot(3,2,2);
    plot(prob1.tvec,prob1.Fxr,'DisplayName',char(lgnd(ii))); box off; hold on;
    ylabel('$F_{x(r)}$ [N]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    p1 = yline(Dxf,'LineStyle','--','DisplayName','Fxf (lim)');
    p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
    p1 = yline(-Dxf,'LineStyle','--','DisplayName','Fxf (lim)');
    p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
    legend('Interpreter','latex');
    
    a(3) = subplot(3,2,3);
    plot(prob1.tvec,prob1.Fyf); box off; hold on;
    ylabel('$F_{y(f)}$ [N]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    yline(Dyf,'LineStyle','--','DisplayName','Fyf (lim)')
    yline(-Dyf,'LineStyle','--','DisplayName','Fyf (lim)')

    a(4) = subplot(3,2,4);
    plot(prob1.tvec,prob1.Fyr); box off; hold on;
    ylabel('$F_{y(r)}$ [N]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    yline(Dyr,'LineStyle','--','DisplayName','Fyf (lim)')
    yline(-Dyr,'LineStyle','--','DisplayName','Fyf (lim)')

    a(5) = subplot(3,2,5);
    plot(prob1.tvec,rad2deg(prob1.alphaf)); box off; hold on;
    ylabel('$\alpha_f$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    a(6) = subplot(3,2,6);
    plot(prob1.tvec,rad2deg(prob1.alphaf)); box off; hold on;
    ylabel('$\alpha_f$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    h = findall(gcf,'Type','axes'); % An array if you have subplots
    set(h, 'TickLabelInterpreter', 'latex')

    linkaxes(a,'x');
    
    figure(figno); figno = figno + 1;

    plot3(prob1.x,prob1.y,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'DisplayName',char(lgnd(ii)),'LineWidth',2);grid on; hold on;
    zlabel('$v$ [km/h]','Interpreter','latex');
    ylabel('$y$ [m]','Interpreter','latex'); xlabel('$x$ [m]','Interpreter','latex');
    if ii == 3 || ii == 4 || ii == 6 || ii == 8
        p1 = fimplicit(@(x,y) path_in(x,y) - 1 ,[0 2*Xfin 0 Ohgt+Owdty],'Color','k','LineStyle','--','LineWidth',2);
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
        p1 = fimplicit(@(x,y) path_out(x,y) - 1 ,[0 2*Xfin 0 Ohgt+Owdty],'Color','k','LineStyle','--','LineWidth',2);
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
        legend('Interpreter','latex');
    end
    h = findall(gcf,'Type','axes'); % An array if you have subplots
    set(h, 'TickLabelInterpreter', 'latex')

    linkaxes(a,'x'); clear a
    
end

% figure(figno-2);
%% functions 
function [x, y] = VehicleShapeNew(x, y, theta, l, w)        
    box = [x-l/2, y+w/2; x+l/2, y+w/2; x+l/2, y-w/2; x-l/2, y-w/2];    
    box_matrix = box - repmat([x, y], size(box, 1), 1);    
    theta = -theta;    
    rota_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];    
    new = box_matrix * rota_matrix + repmat([x, y], size(box, 1), 1);        
    x = [new(1,1), new(2,1), new(3,1), new(4,1), new(1,1)];    
    y = [new(1,2), new(2,2), new(3,2), new(4,2), new(1,2)];
end


