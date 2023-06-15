%% plots for the report 
clear  p 

figure(1); clf;
% tiledlayout(1, 6);    

% ax(1) = nexttile([1 6]);
ax(1) = gca;
val = dir;

textwidth = 14;
textheight = 7;
figsize = [0.3 * textwidth, textheight];

for ii = 1:2
    switch ii 
        case 1 
%             load('init_5m_results.mat');
load('A:\Courses\optimal_vehicle_maneuvers\excercises\pep\interesting_results\hrp_50m_30kmph_t_opt_init_res_v2.mat');
%             lgnd(ii) = "min $t$";
            clr = [0 0.4470 0.7410];
            R2 = 50;
            path_in = @(x,y) ((x-10)/2).^6 + (y/R2).^6;
            path_out = @(x,y) ((x-10)/(2+5)).^6 + (y/(R2+5)).^6;
            path_mid = @(x,y) ((x-10)/(2+5/2)).^6 + (y/(R2+5/2)).^6;
        case 2
%             load('init_10m_results.mat');
load('A:\Courses\optimal_vehicle_maneuvers\excercises\pep\interesting_results\hrp_50m_30kmph_vf_opt_init_res_v2.mat');
%             lgnd(ii) = "max $v_f$";
            clr = [0.8500 0.3250 0.0980];
            R2 = 50;
            path_in = @(x,y) ((x-10)/2).^6 + (y/R2).^6;
            path_out = @(x,y) ((x-10)/(2+5)).^6 + (y/(R2+5)).^6;
            path_mid = @(x,y) ((x-10)/(2+5/2)).^6 + (y/(R2+5/2)).^6;
        case 3 
            load('init_20m_results.mat');
%             lgnd(ii) = "min $t$";
            clr = [0.9290 0.6940 0.1250];
            R2 = 20;
            path_in = @(x,y) ((x-10)/2).^6 + (y/R2).^6;
            path_out = @(x,y) ((x-10)/(2+5)).^6 + (y/(R2+5)).^6;
            path_mid = @(x,y) ((x-10)/(2+5/2)).^6 + (y/(R2+5/2)).^6;
        case 4
            load('init_30m_results.mat');
%             lgnd(ii) = "max $v_f$";
            clr = [0.4940 0.1840 0.5560];    
            R2 = 30;
            path_in = @(x,y) ((x-10)/2).^6 + (y/R2).^6;
            path_out = @(x,y) ((x-10)/(2+5)).^6 + (y/(R2+5)).^6;
            path_mid = @(x,y) ((x-10)/(2+5/2)).^6 + (y/(R2+5/2)).^6;
        case 5 
            load('init_40m_results.mat');
%             lgnd(ii) = "min $t$";
            clr = [0.4660 0.6740 0.1880];
            R2 = 40;
            path_in = @(x,y) ((x-10)/2).^6 + (y/R2).^6;
            path_out = @(x,y) ((x-10)/(2+5)).^6 + (y/(R2+5)).^6;
            path_mid = @(x,y) ((x-10)/(2+5/2)).^6 + (y/(R2+5/2)).^6;
        case 6
            load('init_50m_results.mat');
%             lgnd(ii) = "max $v_f$";
            clr = [0.3010 0.7450 0.9330];
            R2 = 50;
            path_in = @(x,y) ((x-10)/2).^6 + (y/R2).^6;
path_out = @(x,y) ((x-10)/(2+5)).^6 + (y/(R2+5)).^6;
path_mid = @(x,y) ((x-10)/(2+5/2)).^6 + (y/(R2+5/2)).^6;
    end
    
    %--- trajectory plot ---- 
%     ax(1) = nexttile();
    p(ii) = plot(ax(1),prob1.x,prob1.y); box(ax(1),'off');
    hold(ax(1),'on');
    p(ii).Color = clr;
    if ii == 2
        p1 = fimplicit(ax(1),@(x,y) path_in(x,y) - 1 ,[0 2*10 0 55],'Color','k');
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
        p1 = fimplicit(ax(1),@(x,y) path_out(x,y) - 1 ,[0 2*10 0 55],'Color','k');
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
        p1 = fimplicit(ax(1),@(x,y) path_mid(x,y) - 1 ,[0 2*10 0 55],'--','Color','k');
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
    ylabel(ax(1),'$Y_p$ [m]','Interpreter','latex');
    xlabel(ax(1),'$X_p$ [m]','Interpreter','latex');
    xlim(ax(1),[3 17]); ylim(ax(1),[0 55.5]);
    
    t = prob1.tvec;
    
    for jj = 0:1:round(t(end))
        xi = interp1(t,prob1.x,jj); 
        yi = interp1(t,prob1.y,jj);

        yaw = interp1(t,prob1.psi,jj); % yaw at time instant t

        [xval, yval] = VehicleShapeNew(xi,yi,yaw,1.3+1.5,0);

        p1 = plot(ax(1),xval, yval,'LineWidth',2); % Plotting line segment
        p1.Color = clr;
        
        p1.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
    
    h = findall(gcf,'Type','axes'); % An array if you have subplots
    set(h, 'TickLabelInterpreter', 'latex')
    
end

set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

% print -dpdf A:\Courses\optimal_vehicle_maneuvers\ppt\figures\inc_opti_alg.pdf
print -dpdf A:\Courses\optimal_vehicle_maneuvers\ppt\figures\pep1_part1.pdf


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