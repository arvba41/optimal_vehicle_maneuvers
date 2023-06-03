function plottingrest_report_fep_detailed(optres1,optres2,mdl,cons)

% plotting

textwidth = 15.00;
% golden_ratio = (1 + sqrt(5)) / 2;
textheight = 18;
% figsize = [0.9 * textwidth, 0.9 * textwidth / golden_ratio];
figsize = [textwidth, textheight];

figure(21), clf;

% vx
subplot(3,2,1)
plot(optres1.tvec,optres1.vx*3.6,'DisplayName','fric-PM'); hold on;
ylabel('$v_x$ [km/h]','Interpreter','latex','FontSize',10);
xlabel('$t$ [s]','Interpreter','latex','FontSize',10);
box off; 
plot(optres2.tvec,optres2.vx*3.6,'DisplayName','fric-PM'); hold on;
legend('Interpreter','latex');
% xline(optres1.tvec(optres1.int_idx),'-.','HandleVisibility','off');
% xline(optres2.tvec(optres2.int_idx),'--','HandleVisibility','off');

% vy
subplot(3,2,2)
plot(optres1.tvec,optres1.vy*3.6,'DisplayName','fric-PM'); hold on;
ylabel('$v_y$ [km/h]','Interpreter','latex','FontSize',10);
xlabel('$t$ [s]','Interpreter','latex','FontSize',10);
box off; 
plot(optres2.tvec,optres2.vy*3.6,'DisplayName','fric-PM'); hold on;
% legend('Interpreter','latex');
% xline(optres1.tvec(optres1.int_idx),'-.','HandleVisibility','off');
% xline(optres2.tvec(optres2.int_idx),'--','HandleVisibility','off');

% ux,uy
subplot(3,2,3)
yyaxis left
plot(optres1.tvec(2:end),optres1.ux/1e3,'DisplayName','fric-PM'); hold on;
ylabel('$u_x$ [kN]','Interpreter','latex','FontSize',10);
xlabel('$t$ [s]','Interpreter','latex','FontSize',10);
box off; 
plot(optres2.tvec(2:end),optres2.ux/1e3,'DisplayName','rate-PM'); hold on;
yyaxis right
plot(optres1.tvec(2:end),optres1.uy/1e3,'DisplayName','fric-PM','HandleVisibility','off'); hold on;
ylabel('$u_y$ [kN]','Interpreter','latex','FontSize',10);
plot(optres2.tvec(2:end),optres2.uy/1e3,'DisplayName','rate-PM','HandleVisibility','off'); hold on;
legend('Interpreter','latex');
% xline(optres1.tvec(optres1.int_idx),'-.','HandleVisibility','off');
% xline(optres2.tvec(optres2.int_idx),'--','HandleVisibility','off');

% u
subplot(3,2,4)
plot(optres1.tvec(2:end),sqrt(optres1.ux.^2 + optres1.uy.^2)/1e3,'DisplayName','fric-PM'); hold on;
ylabel('$u$ [kN]','Interpreter','latex','FontSize',10);
xlabel('$t$ [s]','Interpreter','latex','FontSize',10);
box off; 
plot(optres2.tvec(2:end),optres2.u/1e3,'DisplayName','rate-PM'); hold on;
% legend('Interpreter','latex');
% xline(optres1.tvec(optres1.int_idx),'-.','HandleVisibility','off');
% xline(optres2.tvec(optres2.int_idx),'--','HandleVisibility','off');
umax = mdl.mu*mdl.mass*mdl.g;
yline(umax/1e3,'--','HandleVisibility','off');

% delta
subplot(3,2,5)
plot(optres2.tvec,rad2deg(optres2.delta),'DisplayName','rate-PM'); hold on;
ylabel('$\delta$ [deg]','Interpreter','latex','FontSize',10);
xlabel('$t$ [s]','Interpreter','latex','FontSize',10);
box off; 
legend('Interpreter','latex');
% xline(optres1.tvec(optres1.int_idx),'-.','HandleVisibility','off');
% xline(optres2.tvec(optres2.int_idx),'--','HandleVisibility','off');
yline(rad2deg(cons.delta_max),'--','HandleVisibility','off');
yline(-rad2deg(cons.delta_max),'--','HandleVisibility','off');

subplot(3,2,6)
plot(optres2.tvec,rad2deg(optres2.ddelta),'DisplayName','rate-PM'); hold on;
ylabel('$\dot\delta$ [deg]','Interpreter','latex','FontSize',10);
xlabel('$t$ [s]','Interpreter','latex','FontSize',10);
box off; 
% legend('Interpreter','latex');
% xline(optres1.tvec(optres1.int_idx),'-.','HandleVisibility','off');
% xline(optres2.tvec(optres2.int_idx),'--','HandleVisibility','off');
yline(rad2deg(cons.ddelta_max),'--','HandleVisibility','off');
yline(-rad2deg(cons.ddelta_max),'--','HandleVisibility','off');

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')
set(gca,'FontSize',10);

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\flp_avoid_detailed.pdf