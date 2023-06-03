function ax = plottingrest_rate(prob,name)

ax(1) = subplot(3,4,1:4);
plot(prob.x,prob.y); 
box off; hold on; 
title('Trajectory','Interpreter','latex');
ylabel('$y$ [m]','Interpreter','latex'); 
xlabel('$x$ [m]','Interpreter','latex');

ax(2) = subplot(3,4,5);
plot(prob.tvec,prob.vx*3.6,'LineWidth',2,'DisplayName',name); 
box off; hold off; 
title('Velocity $x$','Interpreter','latex');
ylabel('$v_x$ [km/h]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')

ax(3) = subplot(3,4,6);
plot(prob.tvec,prob.vy*3.6,'LineWidth',2,'DisplayName',name); 
box off; hold off; 
title('Velocity $y$','Interpreter','latex');
ylabel('$v_y$ [km/h]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')

ax(4) = subplot(3,4,7);
yyaxis left
plot(prob.tvec(2:end),prob.ux,'LineWidth',2,'DisplayName',name); 
ylabel('$F_x$ [N]','Interpreter','latex'); 
yyaxis right
plot(prob.tvec(2:end),prob.uy,'LineWidth',2,'DisplayName',name); 
ylabel('$F_y$ [N]','Interpreter','latex'); 
box off; hold on; 
title('Forces','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')

ax(5) = subplot(3,4,8);
box off; hold on; 
plot(prob.tvec(2:end),prob.u,'LineWidth',2,'DisplayName',name); 
box off; hold on; 
title('Total Force','Interpreter','latex');
ylabel('$F$ [N]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')

ax(6) = subplot(3,4,9);
box off; hold on; 
plot(prob.tvec,rad2deg(prob.delta),'LineWidth',2,'DisplayName',name); 
box off; hold on; 
title('$\delta$','Interpreter','latex');
ylabel('$F$ [N]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')

ax(7) = subplot(3,4,10);
plot(prob.tvec,rad2deg(prob.ddelta),'LineWidth',2,'DisplayName',name); 
box off; hold on; 
title('$\dot\delta$','Interpreter','latex');
ylabel('$F$ [N]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex');
legend('Interpreter','latex')
box off; hold on; 
            
end