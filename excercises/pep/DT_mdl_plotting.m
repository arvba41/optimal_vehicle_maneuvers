textwidth = 15;
textheight = 13;
golden_ratio = (1 + sqrt(5)) / 2;
figsize = [0.6 * textwidth, 0.9 * textheight];

figure(20);

plot(prob1.x-10,prob1.y,'Color','b'); hold on;
haipin_i = @(x,y) (x/2)^4 + (y/50)^4;
haipin_o = @(x,y) (x/7)^4 + (y/55)^4;
fimplicit(@(x,y) haipin_i(x,y) - 1 ,[-7 7 0 55],'Color','k');
fimplicit(@(x,y) haipin_o(x,y) - 1 ,[-7 7 0 55],'Color','k');
ylabel('y [m]','Interpreter','latex');
xlabel('x [m]','Interpreter','latex');

t = prob1.tvec;

for ii = 0:1:round(t(end))
    xi = interp1(t,prob1.x-10,ii); 
    yi = interp1(t,prob1.y,ii);
    
    yaw = interp1(t,prob1.psi,ii); % yaw at time instant t
    
    [xval, yval] = VehicleShapeNew(xi,yi,yaw,lf+lr,w);

    plot(xval, yval, 'b-'); % Plotting line segment
end

hold off; xlim([-7 7]); ylim([0 55]);

set(gca,'FontSize',10);

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\prob2_DT_path.pdf

%% second set of plots
textheight = 25;
figsize = [textwidth, textheight];

figure(22); clf;
nrows = 6;

subplot(nrows,2,1);
plot(prob1.tvec,rad2deg(prob1.r)); box off;
ylabel('$\dot \psi$ [deg/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

subplot(nrows,2,2);
plot(prob1.tvec,rad2deg(wrapToPi(prob1.psi))); box off;
ylabel('$\psi$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

subplot(nrows,2,3);
plot(prob1.tvec,rad2deg(prob1.delta)); box off;
ylabel('$\delta$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')
yline(-rad2deg(deltamax),'LineStyle','--','DisplayName','$\delta$ (lim)')

subplot(nrows,2,4);
plot(prob1.tvec(2:end),rad2deg(prob1.ddelta)); box off;
ylabel('$\dot \delta$ [deg/s]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(rad2deg(ddeltamax),'LineStyle','--','DisplayName','$\dot \delta$ (lim)')
yline(-rad2deg(ddeltamax),'LineStyle','--','DisplayName','$\dot \delta$ (lim)')


subplot(nrows,2,5);
plot(prob1.tvec,prob1.vx*3.6); box off;
ylabel('$v_x$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

subplot(nrows,2,6);
plot(prob1.tvec,prob1.vy*3.6); box off;
ylabel('$v_y$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

subplot(nrows,2,7);
plot(prob1.tvec,prob1.Fxf/1e3); box off; hold on;
plot(prob1.tvec,prob1.Fxr/1e3); box off; hold on;
ylabel('$F_{x}$ [kN]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dxf/1e3,'LineStyle','--','DisplayName','Fxf (lim)')
yline(-Dxf/1e3,'LineStyle','--','DisplayName','Fxf (lim)')
legend('front','rear','Interpreter','latex');

subplot(nrows,2,8);
plot(prob1.tvec,[prob1.Fy1; prob1.Fy2; prob1.Fy3; prob1.Fy4]/1e3); box off;
ylabel('$F_{y}$ [kN]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
yline(Dyf/1e3,'LineStyle','--','DisplayName','Fyf (lim)')
yline(-Dyf/1e3,'LineStyle','--','DisplayName','Fyf (lim)')
legend('1','2','3','4','Interpreter','latex','NumColumns',2);

subplot(nrows,2,9:10);
plot(prob1.tvec,rad2deg([prob1.alpha1; prob1.alpha2; prob1.alpha3; prob1.alpha4])); box off;
ylabel('$\alpha$ [deg]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');
legend('1','2','3','4','Interpreter','latex','NumColumns',2,'Location','southeast');

subplot(nrows,2,11);
plot(prob1.tvec,[prob1.vx1; prob1.vx2; prob1.vx3; prob1.vx4]*3.6); box off;
ylabel('$v_{x}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

subplot(nrows,2,12);
plot(prob1.tvec,[prob1.vy1; prob1.vy2; prob1.vy3; prob1.vy4]*3.6); box off;
ylabel('$v_{y}$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

% Set size and no crop
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', figsize);
set(gcf, 'PaperUnits', 'normalized', 'PaperPosition', [0, 0, 1, 1]);

print -dpdf A:\Courses\optimal_vehicle_maneuvers\doc\figures\prob2_DT_path_detail.pdf
%% funcitons 
function [x, y] = VehicleShapeNew(x, y, theta, l, w)        
    box = [x-l/2, y+w/2; x+l/2, y+w/2; x+l/2, y-w/2; x-l/2, y-w/2];    
    box_matrix = box - repmat([x, y], size(box, 1), 1);    
    theta = -theta;    
    rota_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];    
    new = box_matrix * rota_matrix + repmat([x, y], size(box, 1), 1);        
    x = [new(1,1), new(2,1), new(3,1), new(4,1), new(1,1)];    
    y = [new(1,2), new(2,2), new(3,2), new(4,2), new(1,2)];
end