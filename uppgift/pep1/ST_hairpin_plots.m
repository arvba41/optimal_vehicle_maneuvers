clear all; clc;
%% ---- model parameterization ----

% Pacejka Magic Formula parameters
sel_fric = 1; % selecting tire parameters based on friction
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
params.m = 2100;
params.lf = 1.3; 
params.lr = 1.5;
params.g = 9.82;
params.Izz = 3900; % 25% increase in inertia (case 1 with 50m and 50kph) %% 10% increase in inertia 
params.Iw = 4;
params.Re = 0.3;
params.h = 0.50;

% curve parameters
Ri = 72.5;  % inner radius
Ro = 77.5;  % outer radius
Rm = 75;    % mid point
n = 10;     % degree of the turn
Xa = 75;    % center of the curve 

% steering constraints
deltamax = pi/6;
ddeltamax = pi/4;

% curve
path_in = @(x,y) ((x - Xa)/Ri).^n + (y/Ri).^n;
path_mid = @(x,y) ((x - Xa)/Rm).^n + (y/Rm).^n;
path_out = @(x,y) ((x - Xa)/Ro).^n + (y/Ro).^n;

%% figure 1: comparing the model with LT ans static (position plot)

% path plots 

figure(1); clf;

textwidth = 15;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [textwidth, 0.8 * textwidth / golden_ratio];
for ii = 1:2
    switch ii
        case 1
            load('ST_right_turn\ST_topt\begin_center.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'static';
        case 2
            load('ST_right_turn\ST_LT_topt\begin_center.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'LT';
    end
    
    subplot(121)
    plot(prob1.x,prob1.y,'Color',clr,'DisplayName',str); hold on; grid off; box off;
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    ylabel('$y$ [m]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');

    t = prob1.tvec;

    for kk = 0:1:round(t(end))
        xi = interp1(t,prob1.x,kk); 
        yi = interp1(t,prob1.y,kk);

        yaw = interp1(t,prob1.psi,kk); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,params.lf+params.lr,0);

        plot(xval, yval,'LineWidth',4,'Color',clr,'HandleVisibility','off'); % Plotting line segment
    end

    subplot(122)
    plot3(prob1.x,prob1.y,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr);grid on; hold on;
    plot(prob1.x,prob1.y,'Color',clr,'LineStyle','--'); 
    zlabel('$v$ [km/h]','Interpreter','latex');
    ylabel('$y$ [m]','Interpreter','latex'); xlabel('$x$ [m]','Interpreter','latex');
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    view([90-37.5 30])
%     hold off; % xlim([0 inf])
end
subplot(121)
legend('Interpreter','latex','Location','southeast');
ylim ([0 inf]); xlim([-2.5 inf]);

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

% print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\pep2_comp_posplot.pdf

%% figure 2:  comparing the model with LT ans static (detiled plots)

figure(2); clf;
textwidth   = 15;
textheight  = 15;
figsize = [textwidth, textheight];

for ii = 1:2
    switch ii
        case 1 
            load('ST_right_turn\ST_topt\begin_center.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'static';
            Fzf = params.m * params.g * params.lr / ( params.lf + params.lr );
            Fzr = params.m * params.g * params.lf / ( params.lf + params.lr );
            Dxf = params.mux(1) * Fzf / 1e3; 
            Dxr = params.mux(2) * Fzr / 1e3;
            Dyf = params.muy(1) * Fzf / 1e3; 
            Dyr = params.muy(2) * Fzr / 1e3;
        case 2
            load('ST_right_turn\ST_LT_topt\begin_center.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'LT';
            Dxf = params.mux(1) * prob1.Fzf / 1e3; 
            Dxr = params.mux(2) * prob1.Fzr / 1e3;
            Dyf = params.muy(1) * prob1.Fzf / 1e3;  
            Dyr = params.muy(2) * prob1.Fzr / 1e3;
    end
    
    subplot(521); hold on
    plot(prob1.tvec,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr); box off;
    ylabel('$v$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(522); hold on
    plot(prob1.tvec,rad2deg(prob1.r),'Color',clr); box off;
    ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(523); hold on
    plot(prob1.tvec,rad2deg(prob1.delta),'Color',clr); box off;
    ylabel('$\delta$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); 
%     yline(rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
%     yline(-rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
    
    subplot(524); hold on
    plot(prob1.tvec,prob1.M_Z/1e3,'Color',clr); box off;
    ylabel('$M_{Z}$ [kNm]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(525); hold on
    plot(prob1.tvec,rad2deg(prob1.alphaf),'Color',clr); box off;
    ylabel('$\alpha_f$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(526); hold on
    plot(prob1.tvec,rad2deg(prob1.alphar),'Color',clr); box off;
    ylabel('$\alpha_r$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(527); hold on
    plot(prob1.tvec,prob1.Fxf/1e3,'Color',clr); box off;
    ylabel('$F_{x(f)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dxf,'k','LineStyle','--','DisplayName','Fxf (lim)')
%     plot(prob1.tvec,-Dxf,'k','LineStyle','--','DisplayName','Fxf (lim)')

    subplot(528); hold on
    plot(prob1.tvec,prob1.Fxr/1e3,'Color',clr); box off;
    ylabel('$F_{x(r)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dxr,'k','LineStyle','--','DisplayName','Fxf (lim)')
%     plot(prob1.tvec,-Dxr,'k','LineStyle','--','DisplayName','Fxf (lim)')

    subplot(529); hold on
    plot(prob1.tvec,prob1.Fyf/1e3,'Color',clr); box off;
    ylabel('$F_{y(f)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dyf,'k','LineStyle','--','DisplayName','Fyf (lim)')
%     plot(prob1.tvec,-Dyf,'k','LineStyle','--','DisplayName','Fyf (lim)')

    subplot(5,2,10); hold on
    plot(prob1.tvec,prob1.Fyr/1e3,'Color',clr); box off;
    ylabel('$F_{y(r)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dyr,'k','LineStyle','--','DisplayName','Fyf (lim)')
%     plot(prob1.tvec,-Dyr,'k','LineStyle','--','DisplayName','Fyf (lim)')

end

% legend('Interpreter','latex','Location','southeast');
% ylim ([0 inf]); xlim([-2.5 inf]);

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

% print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\pep2_comp_detailplot.pdf

%% figure 3: topt vs vopt 
% path plots 

figure(3); clf;

textwidth = 15;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [textwidth, 0.8 * textwidth / golden_ratio];
for ii = 1:2
    switch ii
        case 1
            load('ST_right_turn\ST_LT_topt\begin_center.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt';
        case 2
            load('ST_right_turn\ST_LT_vopt\begin_center.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'vopt';
    end
    
    subplot(121)
    plot(prob1.x,prob1.y,'Color',clr,'DisplayName',str); hold on; grid off; box off;
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    ylabel('$y$ [m]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');

    t = prob1.tvec;

    for kk = 0:1:round(t(end))
        xi = interp1(t,prob1.x,kk); 
        yi = interp1(t,prob1.y,kk);

        yaw = interp1(t,prob1.psi,kk); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,params.lf+params.lr,0);

        plot(xval, yval,'LineWidth',4,'Color',clr,'HandleVisibility','off'); % Plotting line segment
    end

    subplot(122)
    plot3(prob1.x,prob1.y,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr);grid on; hold on;
    plot(prob1.x,prob1.y,'Color',clr,'LineStyle','--'); 
    zlabel('$v$ [km/h]','Interpreter','latex');
    ylabel('$y$ [m]','Interpreter','latex'); xlabel('$x$ [m]','Interpreter','latex');
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    view([90-37.5 30])
%     hold off; % xlim([0 inf])
end

subplot(121)
legend('Interpreter','latex','Location','southeast');
ylim ([0 inf]); xlim([-2.5 inf]);

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

% print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\pep2_TVcomp_posplot.pdf

%% figure 4: topt vs vopt (detailed)

figure(4); clf;
textwidth   = 15;
textheight  = 15;
figsize = [textwidth, textheight];

for ii = 1:2
    switch ii
        case 1 
            load('ST_right_turn\ST_LT_topt\begin_center.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'static';
%             Fzf = params.m * params.g * params.lr / ( params.lf + params.lr );
%             Fzr = params.m * params.g * params.lf / ( params.lf + params.lr );
            Dxf = params.mux(1) * prob1.Fzf / 1e3; 
            Dxr = params.mux(2) * prob1.Fzr / 1e3;
            Dyf = params.muy(1) * prob1.Fzf / 1e3; 
            Dyr = params.muy(2) * prob1.Fzr / 1e3;
        case 2
            load('ST_right_turn\ST_LT_vopt\begin_center.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'LT';
            Dxf = params.mux(1) * prob1.Fzf / 1e3; 
            Dxr = params.mux(2) * prob1.Fzr / 1e3;
            Dyf = params.muy(1) * prob1.Fzf / 1e3;  
            Dyr = params.muy(2) * prob1.Fzr / 1e3;
    end
    
    subplot(521); hold on
    plot(prob1.tvec,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr); box off;
    ylabel('$v$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(522); hold on
    plot(prob1.tvec,rad2deg(prob1.r),'Color',clr); box off;
    ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(523); hold on
    plot(prob1.tvec,rad2deg(prob1.delta),'Color',clr); box off;
    ylabel('$\delta$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); 
%     yline(rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
%     yline(-rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
    
    subplot(524); hold on
    plot(prob1.tvec,prob1.M_Z/1e3,'Color',clr); box off;
    ylabel('$M_{Z}$ [kNm]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(525); hold on
    plot(prob1.tvec,rad2deg(prob1.alphaf),'Color',clr); box off;
    ylabel('$\alpha_f$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(526); hold on
    plot(prob1.tvec,rad2deg(prob1.alphar),'Color',clr); box off;
    ylabel('$\alpha_r$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(527); hold on
    plot(prob1.tvec,prob1.Fxf/1e3,'Color',clr); box off;
    ylabel('$F_{x(f)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dxf,'k','LineStyle','--','DisplayName','Fxf (lim)')
%     plot(prob1.tvec,-Dxf,'k','LineStyle','--','DisplayName','Fxf (lim)')

    subplot(528); hold on
    plot(prob1.tvec,prob1.Fxr/1e3,'Color',clr); box off;
    ylabel('$F_{x(r)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dxr,'k','LineStyle','--','DisplayName','Fxf (lim)')
%     plot(prob1.tvec,-Dxr,'k','LineStyle','--','DisplayName','Fxf (lim)')

    subplot(529); hold on
    plot(prob1.tvec,prob1.Fyf/1e3,'Color',clr); box off;
    ylabel('$F_{y(f)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dyf,'k','LineStyle','--','DisplayName','Fyf (lim)')
%     plot(prob1.tvec,-Dyf,'k','LineStyle','--','DisplayName','Fyf (lim)')

    subplot(5,2,10); hold on
    plot(prob1.tvec,prob1.Fyr/1e3,'Color',clr); box off;
    ylabel('$F_{y(r)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dyr,'k','LineStyle','--','DisplayName','Fyf (lim)')
%     plot(prob1.tvec,-Dyr,'k','LineStyle','--','DisplayName','Fyf (lim)')

end

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\pep2_TVcomp_detailplot.pdf

%% figure 5: effect of friction 
% path plots 

figure(5); clf;

textwidth = 15;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [textwidth, 2 * 0.8 * textwidth / golden_ratio];

subplot(221)
legend('Interpreter','latex','Location','southeast');
ylim ([0 inf]); xlim([-2.5 inf]);

for ii = 1:2
    switch ii
        case 1
            load('ST_right_turn\ST_LT_fric\topt_wet.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt';
        case 2
            load('ST_right_turn\ST_LT_fric\vopt_wet.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'vopt';
    end
    
    subplot(221)
    plot(prob1.x,prob1.y,'Color',clr,'DisplayName',str); hold on; grid off; box off;
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    ylabel('$y$ [m]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');

    t = prob1.tvec;

    for kk = 0:1:round(t(end))
        xi = interp1(t,prob1.x,kk); 
        yi = interp1(t,prob1.y,kk);

        yaw = interp1(t,prob1.psi,kk); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,params.lf+params.lr,0);

        plot(xval, yval,'LineWidth',4,'Color',clr,'HandleVisibility','off'); % Plotting line segment
    end
    
    title('wet asphalt','Interpreter','latex');

    subplot(222)
    plot3(prob1.x,prob1.y,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr);grid on; hold on;
    plot(prob1.x,prob1.y,'Color',clr,'LineStyle','--'); 
    zlabel('$v$ [km/h]','Interpreter','latex');
    ylabel('$y$ [m]','Interpreter','latex'); xlabel('$x$ [m]','Interpreter','latex');
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    view([90-37.5 30])
%     hold off; % xlim([0 inf])
end

subplot(221)
legend('Interpreter','latex','Location','southeast');
ylim ([0 inf]); xlim([-2.5 inf]);

for ii = 1:2
    switch ii
        case 1
            load('ST_right_turn\ST_LT_fric\topt_snow.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt';
        case 2
            load('ST_right_turn\ST_LT_fric\vopt_snow.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'vopt';
    end
    
    subplot(223)
    plot(prob1.x,prob1.y,'Color',clr,'DisplayName',str); hold on; grid off; box off;
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    ylabel('$y$ [m]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');

    t = prob1.tvec;
    
    title('snow','Interpreter','latex');

    for kk = 0:1:round(t(end))
        xi = interp1(t,prob1.x,kk); 
        yi = interp1(t,prob1.y,kk);

        yaw = interp1(t,prob1.psi,kk); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,params.lf+params.lr,0);

        plot(xval, yval,'LineWidth',4,'Color',clr,'HandleVisibility','off'); % Plotting line segment
    end

    subplot(224)
    plot3(prob1.x,prob1.y,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr);grid on; hold on;
    plot(prob1.x,prob1.y,'Color',clr,'LineStyle','--'); 
    zlabel('$v$ [km/h]','Interpreter','latex');
    ylabel('$y$ [m]','Interpreter','latex'); xlabel('$x$ [m]','Interpreter','latex');
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    view([90-37.5 30])
%     hold off; % xlim([0 inf])
end

subplot(223)
% legend('Interpreter','latex','Location','southeast');
ylim ([0 inf]); xlim([-2.5 inf]);

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

% print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\pep2_FricComp_posplot.pdf

%% figure 6: effect of read friction (detailed)

figure(6); clf;
textwidth   = 15;
textheight  = 15;
figsize = [textwidth, textheight];

for ii = 1:4
    switch ii
        case 1 
            load('ST_right_turn\ST_LT_fric\topt_wet.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt-wet';
%             Fzf = params.m * params.g * params.lr / ( params.lf + params.lr );
%             Fzr = params.m * params.g * params.lf / ( params.lf + params.lr );
            Dxf = params.mux(1) * prob1.Fzf / 1e3; 
            Dxr = params.mux(2) * prob1.Fzr / 1e3;
            Dyf = params.muy(1) * prob1.Fzf / 1e3; 
            Dyr = params.muy(2) * prob1.Fzr / 1e3;
        case 2
            load('ST_right_turn\ST_LT_fric\topt_snow.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'topt-snow';
            Dxf = params.mux(1) * prob1.Fzf / 1e3; 
            Dxr = params.mux(2) * prob1.Fzr / 1e3;
            Dyf = params.muy(1) * prob1.Fzf / 1e3;  
            Dyr = params.muy(2) * prob1.Fzr / 1e3;
        case 3 
            load('ST_right_turn\ST_LT_fric\vopt_wet.mat'); 
            clr = [0.9290 0.6940 0.1250];
            str = 'vopt-wet';
%             Fzf = params.m * params.g * params.lr / ( params.lf + params.lr );
%             Fzr = params.m * params.g * params.lf / ( params.lf + params.lr );
            Dxf = params.mux(1) * prob1.Fzf / 1e3; 
            Dxr = params.mux(2) * prob1.Fzr / 1e3;
            Dyf = params.muy(1) * prob1.Fzf / 1e3; 
            Dyr = params.muy(2) * prob1.Fzr / 1e3;
        case 4
            load('ST_right_turn\ST_LT_fric\vopt_snow.mat'); 
            clr = [0.4940 0.1840 0.5560];
            str = 'vopt-snow';
            Dxf = params.mux(1) * prob1.Fzf / 1e3; 
            Dxr = params.mux(2) * prob1.Fzr / 1e3;
            Dyf = params.muy(1) * prob1.Fzf / 1e3;  
            Dyr = params.muy(2) * prob1.Fzr / 1e3;
    end
    
    subplot(521); hold on
    plot(prob1.tvec,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr); box off;
    ylabel('$v$ [km/h]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(522); hold on
    plot(prob1.tvec,rad2deg(prob1.r),'Color',clr,'DisplayName',str); box off;
    ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(523); hold on
    plot(prob1.tvec,rad2deg(prob1.delta),'Color',clr); box off;
    ylabel('$\delta$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); 
%     yline(rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
%     yline(-rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
    
    subplot(524); hold on
    plot(prob1.tvec,prob1.M_Z/1e3,'Color',clr); box off;
    ylabel('$M_{Z}$ [kNm]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(525); hold on
    plot(prob1.tvec,rad2deg(prob1.alphaf),'Color',clr); box off;
    ylabel('$\alpha_f$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(526); hold on
    plot(prob1.tvec,rad2deg(prob1.alphar),'Color',clr); box off;
    ylabel('$\alpha_r$ [deg]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex');
    
    subplot(527); hold on
    plot(prob1.tvec,prob1.Fxf/1e3,'Color',clr); box off;
    ylabel('$F_{x(f)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dxf,'k','LineStyle','--','DisplayName','Fxf (lim)')
%     plot(prob1.tvec,-Dxf,'k','LineStyle','--','DisplayName','Fxf (lim)')

    subplot(528); hold on
    plot(prob1.tvec,prob1.Fxr/1e3,'Color',clr,'DisplayName',str); box off;
    ylabel('$F_{x(r)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dxr,'k','LineStyle','--','DisplayName','Fxf (lim)')
%     plot(prob1.tvec,-Dxr,'k','LineStyle','--','DisplayName','Fxf (lim)')

    subplot(529); hold on
    plot(prob1.tvec,prob1.Fyf/1e3,'Color',clr,'DisplayName',str); box off;
    ylabel('$F_{y(f)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dyf,'k','LineStyle','--','DisplayName','Fyf (lim)')
%     plot(prob1.tvec,-Dyf,'k','LineStyle','--','DisplayName','Fyf (lim)')

    subplot(5,2,10); hold on
    plot(prob1.tvec,prob1.Fyr/1e3,'Color',clr); box off;
    ylabel('$F_{y(r)}$ [kN]','Interpreter','latex');
    xlabel('$t$ [s]','Interpreter','latex'); hold on;
%     plot(prob1.tvec,Dyr,'k','LineStyle','--','DisplayName','Fyf (lim)')
%     plot(prob1.tvec,-Dyr,'k','LineStyle','--','DisplayName','Fyf (lim)')

end

subplot(528);
legend('Interpreter','latex','NumColumns',1,'Location','northeast');

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

% print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\pep2_FricComp_detailplot.pdf

%% figure 7: topt vs vopt 
% path plots 

figure(7); clf;

textwidth = 15;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [textwidth, 0.8 * textwidth / golden_ratio];
for ii = 1:2
    switch ii
        case 1
            load('ST_right_turn\ST_LT_curve_deg\COG_0m_Izzinc_wet.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt';
        case 2
            load('ST_right_turn\ST_LT_vopt\begin_center.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'vopt';
    end
    
    subplot(121)
    plot(prob1.x,prob1.y,'Color',clr,'DisplayName',str); hold on; grid off; box off;
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    ylabel('$y$ [m]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');

    t = prob1.tvec;

    for kk = 0:1:round(t(end))
        xi = interp1(t,prob1.x,kk); 
        yi = interp1(t,prob1.y,kk);

        yaw = interp1(t,prob1.psi,kk); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,params.lf+params.lr,0);

        plot(xval, yval,'LineWidth',4,'Color',clr,'HandleVisibility','off'); % Plotting line segment
    end

    subplot(122)
    plot3(prob1.x,prob1.y,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr);grid on; hold on;
    plot(prob1.x,prob1.y,'Color',clr,'LineStyle','--'); 
    zlabel('$v$ [km/h]','Interpreter','latex');
    ylabel('$y$ [m]','Interpreter','latex'); xlabel('$x$ [m]','Interpreter','latex');
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    view([90-37.5 30])
%     hold off; % xlim([0 inf])
end

subplot(121)
legend('Interpreter','latex','Location','southeast');
ylim ([0 inf]); xlim([-2.5 inf]);

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

% print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\pep2_TVcomp_posplot.pdf

%% figure 8: ppt plots

% path plots 

figure(8); clf;

textwidth = 15;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [textwidth, 0.82 * textwidth / golden_ratio];
for ii = 1:2
    switch ii
        case 1
            load('ST_right_turn\ST_LT_fric\topt_snow.mat'); 
            clr = [0 0.4470 0.7410];
            str = 'topt';
        case 2
            load('ST_right_turn\ST_LT_fric\vopt_snow.mat'); 
            clr = [0.8500 0.3250 0.0980];
            str = 'vopt';
    end
    
    subplot(121)
    plot(prob1.x,prob1.y,'Color',clr,'DisplayName',str); hold on; grid off; box off;
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    ylabel('$y$ [m]','Interpreter','latex');
    xlabel('$x$ [m]','Interpreter','latex');

    t = prob1.tvec;

    for kk = 0:1:round(t(end))
        xi = interp1(t,prob1.x,kk); 
        yi = interp1(t,prob1.y,kk);

        yaw = interp1(t,prob1.psi,kk); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,params.lf+params.lr,0);

        plot(xval, yval,'LineWidth',4,'Color',clr,'HandleVisibility','off'); % Plotting line segment
    end

    subplot(122)
    plot3(prob1.x,prob1.y,sqrt(prob1.vx.^2 + prob1.vy.^2)*3.6,'Color',clr);grid on; hold on;
    plot(prob1.x,prob1.y,'Color',clr,'LineStyle','--'); 
    zlabel('$v$ [km/h]','Interpreter','latex');
    ylabel('$y$ [m]','Interpreter','latex'); xlabel('$x$ [m]','Interpreter','latex');
    if ii == 2
        fimplicit(@(x,y) path_in(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_out(x,y) - 1 ,[-2.5 75 0 77.5],'-','Color','k','HandleVisibility','off');
        fimplicit(@(x,y) path_mid(x,y) - 1 ,[-2.5 75 0 77.5],'--','Color','k','HandleVisibility','off');
    end
    view([90-37.5 30])
%     hold off; % xlim([0 inf])
end
subplot(121)
legend('Interpreter','latex','Location','southeast');
ylim ([0 inf]); xlim([-2.5 inf]);

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\ppt\figures\pep2_iceice.pdf

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