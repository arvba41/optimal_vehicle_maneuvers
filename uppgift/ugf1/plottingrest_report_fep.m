function plottingrest_report_fep(optres1,optres2,cons)

% plotting

textwidth = 15.00;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [0.7 * textwidth, 0.7 * textwidth / golden_ratio];

figure(20), clf;
plot(optres1.x,optres1.y,'DisplayName','fric-PM'); hold on;
ylabel('$y$ [m]','Interpreter','latex','FontSize',10);
xlabel('$x$ [m]','Interpreter','latex','FontSize',10);
set(gca,'YGrid','on','GridLineStyle','--','GridColor','k');  
box off; 
plot(optres2.x,optres2.y,'DisplayName','rate-PM'); hold on;
obst = @(x,y) (-((x-cons.Xa)/cons.R1)^cons.pow - (y/cons.R2)^cons.pow + 1 );
fimplicit(@(x,y) obst(x,y),[0 2*50 0 2],'Color','k','LineStyle','--','HandleVisibility','off');
% yline(2,'LineStyle','--'); yline(0,'LineStyle','--'); hold off;
legend('Interpreter','latex');

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')
set(gca,'FontSize',10);

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\flp_avoid.pdf