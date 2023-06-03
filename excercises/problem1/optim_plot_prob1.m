function optim_plot_prob1(prob)

subplot(2,3,1:3)
plot(prob.x,prob.y,'LineWidth',3);
title('Trajectory','Interpreter','latex');
ylabel('$y$ [m]','Interpreter','latex'); 
xlabel('$x$ [m]','Interpreter','latex');
box off;

subplot(2,3,4)
plot(prob.tvec,prob.x,'LineWidth',2,'DisplayName','$x$'); 
hold on
plot(prob.tvec,prob.y,'LineWidth',2,'DisplayName','$y$'); 
hold off
ylabel('distance [m]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')
box off;

subplot(2,3,5)
plot(prob.tvec,prob.vx,'LineWidth',2,'DisplayName','$v_x$'); 
hold on
plot(prob.tvec,prob.vy,'LineWidth',2,'DisplayName','$v_y$'); 
hold off
ylabel('speed [m/s]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')
box off;

subplot(2,3,6)
plot(prob.tvec(2:end),prob.ux,'LineWidth',2,'DisplayName','$u_x$'); 
hold on
plot(prob.tvec(2:end),prob.uy,'LineWidth',2,'DisplayName','$u_y$'); 
hold off
ylabel('Force [N]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
if ~isnan(prob.cons.Flim)
    yline(prob.cons.Flim,'LineStyle','--','LineWidth',2,...
        'Color','k','DisplayName','$u_{lim}$');
end
legend('Interpreter','latex');
box off;

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')


end
