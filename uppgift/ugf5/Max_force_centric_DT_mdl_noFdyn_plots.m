clear all; 
clc; 

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
    case 3  % Snow väg
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
Xa      = 20;
Xend    = Xa - Xsrt;
Ystrt   = 0;
Yfin    = 1;

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

% ---- obstacle ----
obst = @(x,y) ((x-Xa)/R1).^n + (y/R2).^n;


%% figure 1: comparing the model with LT ans static (position plot)

% path plots 

figure(1); clf;

textwidth = 15;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [0.8 * textwidth, 0.8 * textwidth / golden_ratio];
for ii = 1:3
    switch ii
        case 1
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_topt_v2.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt';
        case 2
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_Fcyopt_v2.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'Fcopt';
        case 3
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_Fyopt_v2.mat'); 
            clr = [0.4940 0.1840 0.5560];
            str = 'Fvopt';
    end
    
   
    plot(prob1.x,prob1.y,'Color',clr,'DisplayName',str); hold on; grid off; box off;
    if ii == 2
        fimplicit(@(x,y) obst(x,y) - 1 ,[Xsrt Xend 0 R2+1],'-','Color','k','HandleVisibility','off');
    end
    ylabel('$y$ [m]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');

    t = prob1.tvec;

    for kk = 0:0.25:round(t(end))
        xi = interp1(t,prob1.x,kk); 
        yi = interp1(t,prob1.y,kk);

        yaw = interp1(t,prob1.psi,kk); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,(params.lf+params.lr)*0.5,0);

        plot(xval, yval,'LineWidth',2,'Color',clr,'HandleVisibility','off'); % Plotting line segment
    end

end
legend('Interpreter','latex','Location','northwest');
ylim ([0 inf]); xlim([0 Xend]);

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\prob6_traj.pdf
% print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\prob6_traj.pdf
%% figure(2): the different forces acting on the vehicle

figure(2); clf; 

textwidth = 15;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [textwidth, 0.5 * textwidth / golden_ratio];

for ii = 1:3
    switch ii
        case 1
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_topt.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt';
        case 2
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_Fcyopt_v2.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'Fcopt';
        case 3
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_Fyopt_v2.mat'); 
            clr = [0.4940 0.1840 0.5560];
            str = 'Fvopt';
    end
    
%     subplot(221);
%     plot(prob1.x,prob1.F_X/1e3,'Color',clr); box off; hold on
%     ylabel('$F_{X}$ [kN]','Interpreter','latex');
%     xlabel('$x$ [m]','Interpreter','latex');
%     % yline(Dxf*2 + Dxr*2,'LineStyle','--','DisplayName','Fxf (lim)')
%     % yline(-(Dxf*2 + Dxr*2),'LineStyle','--','DisplayName','Fxf (lim)')
% 
%     subplot(222);
%     plot(prob1.x,prob1.F_Y/1e3,'Color',clr); box off; hold on
%     ylabel('$F_{Y}$ [kN]','Interpreter','latex');
%     xlabel('$x$ [m]','Interpreter','latex');
%     % yline(Dyf*2 + Dyr*2,'LineStyle','--','DisplayName','Fxf (lim)')
%     % yline(-(Dyf*2 + Dyr*2),'LineStyle','--','DisplayName','Fxf (lim)')

%     subplot(323);
%     plot(prob1.x(2:end),prob1.Fpx/1e3,'Color',clr); box off; hold on
%     ylabel('$F_{X(p)}$ [kN]','Interpreter','latex');
%     xlabel('$x$ [m]','Interpreter','latex');
%     % yline(Dxf*2 + Dxr*2,'LineStyle','--','DisplayName','Fxf (lim)')
%     % yline(-(Dxf*2 + Dxr*2),'LineStyle','--','DisplayName','Fxf (lim)')
% 
%     subplot(324);
%     plot(prob1.x(2:end),prob1.Fpy/1e3,'Color',clr); box off; hold on
%     ylabel('$F_{Y(p)}$ [kN]','Interpreter','latex');
%     xlabel('$x$ [m]','Interpreter','latex');
%     % yline(Dyf*2 + Dyr*2,'LineStyle','--','DisplayName','Fxf (lim)')
%     % yline(-(Dyf*2 + Dyr*2),'LineStyle','--','DisplayName','Fxf (lim)')

    subplot(121);
    plot(prob1.x(2:end),prob1.Fcx/1e3,'Color',clr); box off; hold on
    ylabel('$F_{c,x}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    % yline(Dxf*2 + Dxr*2,'LineStyle','--','DisplayName','Fxf (lim)')
    % yline(-(Dxf*2 + Dxr*2),'LineStyle','--','DisplayName','Fxf (lim)')

    subplot(122);
    plot(prob1.x(2:end),prob1.Fcy/1e3,'Color',clr); box off; hold on
    ylabel('$F_{c,y}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    % yline(Dyf*2 + Dyr*2,'LineStyle','--','DisplayName','Fxf (lim)')
    % yline(-(Dyf*2 + Dyr*2),'LineStyle','--','DisplayName','Fxf (lim)')        

end

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\prob6_forces.pdf

%% figure 3: detailed plot for the OCPs

figure(3); clf; 

textwidth = 15;
textheight = 18;
figsize = [textwidth, textheight];

for ii = 1:3
    switch ii
        case 1
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_topt.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt';
        case 2
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_Fcyopt.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'Fcopt';
        case 3
            load('DT_obst_avoid\DT_obst_avoid_20mx3m_Fyopt.mat'); 
            clr = [0.4940 0.1840 0.5560];
            str = 'Fvopt';
    end
    
    subplot(5,4,1:2)
    plot(prob1.x,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr); box off; hold on;
    ylabel('$v$ [km/h]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,3:4)
    plot(prob1.x,rad2deg(prob1.r),'Color',clr); box off; hold on;
    ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,5:6)
    plot(prob1.x,rad2deg(prob1.delta),'Color',clr); box off; hold on;
    ylabel('$\delta$ [deg]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,7:8)
    plot(prob1.x,prob1.M_Z/1e3,'Color',clr); box off; hold on;
    ylabel('$M_{Z}$ [kNm]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,9)
    plot(prob1.x,rad2deg(prob1.alpha1),'Color',clr); box off; hold on;
    ylabel('$\alpha_i$ [deg]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    title('wheel 1','Interpreter','latex');

    subplot(5,4,10)
    plot(prob1.x,rad2deg(prob1.alpha2),'Color',clr); box off; hold on;
%     ylabel('$\alpha_2$ [deg]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    title('wheel 2','Interpreter','latex');
    
    subplot(5,4,11)
    plot(prob1.x,rad2deg(prob1.alpha3,'Color',clr)); box off; hold on;
%     ylabel('$\alpha_3$ [deg]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    title('wheel 3','Interpreter','latex');
    
    subplot(5,4,12)
    plot(prob1.x,rad2deg(prob1.alpha4),'Color',clr); box off; hold on;
%     ylabel('$\alpha_4$ [deg]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    title('wheel 4','Interpreter','latex');
    
    subplot(5,4,13)
    plot(prob1.x,rad2deg(prob1.Fy1)/1e3,'Color',clr); box off; hold on;
    ylabel('$F_{y(i)}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,14)
    plot(prob1.x,rad2deg(prob1.Fy2)/1e3,'Color',clr); box off; hold on;
%     ylabel('$F_{y2}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,15)
    plot(prob1.x,rad2deg(prob1.Fy3)/1e3,'Color',clr); box off; hold on;
%     ylabel('$F_{y3}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,16)
    plot(prob1.x,rad2deg(prob1.Fy4)/1e3,'Color',clr); box off; hold on;
%     ylabel('$F_{y4}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
   
    subplot(5,4,17)
    plot(prob1.x,prob1.Fxf/1e3,'Color',clr); box off; hold on;
    ylabel('$F_{x(i)}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,18)
    plot(prob1.x,prob1.Fxf/1e3,'Color',clr); box off; hold on;
%     ylabel('$F_{x2}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,19)
    plot(prob1.x,prob1.Fxr/1e3,'Color',clr); box off; hold on;
%     ylabel('$F_{x3}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');
    
    subplot(5,4,20)
    plot(prob1.x,prob1.Fxr/1e3,'Color',clr); box off; hold on;
%     ylabel('$F_{x4}$ [kN]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');

end

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\prob6_detailed.pdf

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