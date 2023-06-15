function plotting3(prog)

figure(2); clf;

tiledlayout(2,2);

ax(1) = nexttile(1:2);
plot(prog{1}.x,prog{1}.y,'LineWidth',2,'DisplayName','Problem 1'); 
box off; hold on; 
plot(prog{2}.x,prog{2}.y,'LineWidth',2,'DisplayName','Problem 2'); 
title('Trajectory','Interpreter','latex');
ylabel('$y$ [m]','Interpreter','latex'); 
xlabel('$x$ [m]','Interpreter','latex');
legend('Interpreter','latex')
plot(prog{2}.x,prog{2}.cons.yobst,'LineWidth',3,'LineStyle','--','Color','k');
% plot(prog{2}.x,prog{2}.cons.yllim,'LineWidth',3,'LineStyle','--','Color','k');
% plot(prog{2}.x,prog{2}.cons.yulim,'LineWidth',3,'LineStyle','--','Color','k');

ax(2) = nexttile;
yyaxis left 
plot(prog{1}.tvec(2:end),prog{1}.ux,'LineWidth',2,'DisplayName','Problem 1'); 
hold on;
plot(prog{2}.tvec(2:end),prog{2}.ux,'LineWidth',2,'DisplayName','Problem 2'); 
ylabel('$u_x$ [N]','Interpreter','latex');
yyaxis right
plot(prog{1}.tvec(2:end),prog{1}.uy,'LineWidth',2,'DisplayName','Problem 1'); 
hold on;
plot(prog{2}.tvec(2:end),prog{2}.uy,'LineWidth',2,'DisplayName','Problem 2'); 
hold off
ylabel('$u_y$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')
box off;

ax(3) = nexttile;
plot(prog{1}.tvec(2:end),sqrt(prog{1}.ux.^2 + prog{1}.uy.^2),'LineWidth',2,'DisplayName','Problem 1'); 
hold on;
plot(prog{2}.tvec(2:end),sqrt(prog{2}.ux.^2 + prog{2}.uy.^2),'LineWidth',2,'DisplayName','Problem 2'); 
ylabel('$\sqrt{u_x^2 + u_y^2}$ [N]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')
box off;
yline(prog{2}.cons.Flim,'LineStyle','--','LineWidth',2,...
        'Color','k','DisplayName','$u_{max}$');

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')
end