function potting_simresults_forceXY(t,y,figno,parmas)

figure(figno); clf;

tiledlayout(4,2)

%% plot number 1
ax(1) = nexttile([2 2]); 

plot(y(:,1),y(:,2)); 
xlabel('$x$ [m]','Interpreter','latex')
ylabel('$y$ [m]','Interpreter','latex')
title('vehicle trajectory','Interpreter','latex'); 

hold on; % Hold the plot to add line segments

if figno == 2
    scale = 1;
else
    scale = 4;
end
lf = parmas.lf*scale; % front length of the vehicle from CoM
lr = parmas.lr*scale; % rear length of the vehicle from CoM

yaw_vec = cumtrapz(t,y(:,5)); % determining the yaw angle
% tseg = linspace(1,t(end),5); % time steps for line segment

for ii = linspace(1,t(end),10)
    xi = interp1(t,y(:,1),ii); 
    yi = interp1(t,y(:,2),ii);
    
    yaw = interp1(t,yaw_vec,ii); % yaw at time instant t
    
    xstart = xi - lr*(cos(yaw)); % x-coordinate of line segment startpoint
    xend = xi + lf*(cos(yaw)); % x-coordinate of line segment endpoint
    ystart = yi - lr*(sin(yaw)); % x-coordinate of line segment startpoint
    yend = yi + lf*(sin(yaw)); % x-coordinate of line segment endpoint
    
    plot([xstart, xend], [ystart, yend], 'r-','LineWidth',4); % Plotting line segment
end

hold off; % xlim([0 inf])

%% plot number 2
ax(2) = nexttile; 
plot(t,y(:,3)*3.6,'LineWidth',2); box off
ylabel('$v_x$ [km/h]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex'); 

%% plot number 3
ax(3) = nexttile;
plot(t,y(:,4)*3.6,'LineWidth',2); box off
ylabel('$v_y$ [km/h]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex'); 

%% plot number 4
ax(4) = nexttile;
plot(t,rad2deg(y(:,5)),'LineWidth',2); box off
ylabel('$\dot\psi$ [rad/s]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex'); 

%% plot number 5
ax(5) = nexttile;
plot(t,rad2deg(yaw_vec),'LineWidth',2); box off
ylabel('$\psi$ [deg]','Interpreter','latex'); 
xlabel('$t$ [s]','Interpreter','latex'); 

% %% plot number 6
% ax(6) = nexttile;
% plot(t,y(:,6)*pi/30,'LineWidth',2); box off
% ylabel('$\omega_f$ [rpm]','Interpreter','latex'); 
% xlabel('$t$ [s]','Interpreter','latex'); 
% 
% %% plot number 7
% ax(7) = nexttile;
% plot(t,y(:,7)*pi/30,'LineWidth',2); box off
% ylabel('$\omega_r$ [rpm]','Interpreter','latex'); 
% xlabel('$t$ [s]','Interpreter','latex'); 

% %% plot number 8 and 9 (input singla plots)
% 
% D_strt      = inputs(1);
% D_end       = inputs(2);
% delta_max   = inputs(3);
% T_strt      = inputs(4);
% T_end       = inputs(5);
% Tmax        = inputs(6);
% 
% % ---- input functions ----
% Hsigmoid = @(x) 0.5*(1+tanh(2*pi*6*(x)/(10)));
% Ppulse = @(x,puls_strt,puls_end) Hsigmoid(x-puls_strt) + Hsigmoid(puls_end-x) - 1;
% 
% if figno == 2
%     Tin = @(x,puls_strt,puls_end) Tmax*Ppulse(x,puls_strt,puls_end).*sin(pi*(x)/10 + pi/2);
% else
%     Tin = @(x,puls_strt,puls_end) Tmax*Ppulse(x,puls_strt,puls_end);
% end
% 
% if figno == 3
%     deltain = @(x,puls_strt,puls_end) sin(2*pi*x/10).*Ppulse(x,puls_strt,puls_end)*delta_max;
% else
%     deltain = @(x,puls_strt,puls_end) cos(2*pi*(0)/10).*Ppulse(x,puls_strt,puls_end)*delta_max;
% end
% 
% ax(6) = nexttile;
% plot(t,Tin(t,T_strt,T_end)/1e3,'LineWidth',2); box off
% ylabel('$F_f$ [kN]','Interpreter','latex'); 
% xlabel('$t$ [s]','Interpreter','latex'); 
% 
% ax(7) = nexttile;
% plot(t,rad2deg(deltain(t,D_strt,D_end)),'LineWidth',2); box off
% ylabel('$\delta_f$ [deg]','Interpreter','latex'); 
% xlabel('$t$ [s]','Interpreter','latex'); 

%% linking axes and latexing 
linkaxes(ax(2:end),'x');

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

end