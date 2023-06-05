%% plots
figure(10); clf; 

textwidth = 15;
textheight = 13;
figsize = [textwidth, textheight];

idx = find(round(elps(oplsol.x,oplsol.y),1) == 0);
idx(1) = [];

subplot(3,2,1:2); 
plot(oplsol.x,oplsol.y); box off; hold on;
ylabel('$y$ [m]','Interpreter','latex'); 
xlabel('$x$ [m]','Interpreter','latex');
fimplicit(@(x,y) elps(x,y),[0 25 0 4],'--','Color','k');
xlim([0 oplsol.x(idx)]);

subplot(323); 
yyaxis left
plot(oplsol.tvec,oplsol.vx*3.6); box off;
ylabel('$v_x$ [km/h]','Interpreter','latex'); 
yyaxis right
plot(oplsol.tvec,oplsol.vy*3.6); box off;
ylabel('$v_y$ [km/h]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
xlim([0 oplsol.tvec(idx)]);

subplot(324); 
plot(oplsol.tvec,sqrt(oplsol.vx.^2 + oplsol.vy.^2)*3.6); box off;
ylabel('$v$ [km/h]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
xlim([0 oplsol.tvec(idx)]);

subplot(325); 
yyaxis left
plot(oplsol.tvec(2:end),oplsol.ux/1e3); box off;
ylabel('$F_x$ [kN]','Interpreter','latex'); 
yyaxis right
plot(oplsol.tvec(2:end),oplsol.uy/1e3); box off;
ylabel('$F_y$ [kN]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
xlim([0 oplsol.tvec(idx)]);

subplot(326); 
plot(oplsol.tvec(2:end),sqrt(oplsol.ux.^2 + oplsol.uy.^2)/1e3); box off;
ylabel('$F$ [kN]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
yline(oplsol.Fmax/1e3,'--')
xlim([0 oplsol.tvec(idx)]);
ytickformat('%.1f');

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\prob4_opt_avoid_path.pdf