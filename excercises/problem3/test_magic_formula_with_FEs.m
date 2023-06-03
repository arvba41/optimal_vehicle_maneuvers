clear all; clc;
%% Pacejka Magic Formula with Friction Elipse
% ---- model parameterization ----
params.mux = [1.2 1.2];
params.Bx = [11.7 11.1];
params.Cx = [1.69 1.69];
params.Ex = [0.377 0.362];
params.muy = [0.935 0.961];
params.By = [8.86 9.30];
params.Cy = [1.19 1.19];
params.Ey = [-1.21 -1.11];

params.m = 2100;
params.lf = 1.3; 
params.lr = 1.5;
params.g = 9.8;
% ---- variables ----
alphaf = linspace(-pi/6,pi/6,51); alphar = alphaf;
kf = linspace(-1,1,51); kr = kf;

Fzf = params.m*params.g*params.lr/(params.lf+params.lr);
Fzr = params.m*params.g*params.lf/(params.lf+params.lr);

% ---- Test equation ----
for jj = 1:length(kf)
    for ii = 1:length(alphaf)
        [~,Fxf(ii,jj),Fyf(ii,jj),Fxr(ii,jj),Fyr(ii,jj)] = magic_formula_friction_elipses(params,Fzf,Fzr,alphaf(ii),alphar(ii),kf(jj),kr(jj),100);
    end
end

%% ---- plotting the results ----
figure(1); clf;

ax(1) = subplot(2,2,1);
plot(kf,[Fxf(25,:); Fxr(25,:)]./[Fzf; Fzr],'LineWidth',2); box off;
ylabel('$F_{x,i}/F_z$ [-]','Interpreter','latex'); 
xlabel('$\kappa$ [-]','Interpreter','latex'); 
legend('front','rear','Interpreter','latex');
title('The nominal tire forces, ($\alpha = 0$)',...
    'Interpreter','latex');

ax(2) = subplot(2,2,2);
plot(rad2deg(alphaf),[Fyf(:,25) Fyr(:,25)]./[Fzf Fzr],'LineWidth',2); box off;
ylabel('$F_{y,i}/F_z$ [-]','Interpreter','latex'); 
xlabel('$\alpha$ [deg]','Interpreter','latex'); 
legend('front','rear','Interpreter','latex');
title('The nominal tire forces ($\kappa = 0$)',...
    'Interpreter','latex');

ax(3) = subplot(2,2,3);
for jj = 1:length(kf)
    for ii = 1:length(alphaf)
        val1(ii,jj) = Fxf(ii,jj);
        val2(ii,jj) = Fyf(ii,jj);
    end
end
surf(kf,rad2deg(alphaf),sqrt(val1.^2 + val2.^2)./Fzf)
ylabel('$\alpha_f$ [deg]','Interpreter','latex'); 
xlabel('$\kappa_f$ [-]','Interpreter','latex'); 
zlabel('$\sqrt{F_{x,r}^2 + F_{y,r}^2}/F_{z,r}$ [-]','Interpreter','latex'); 
title('Resulting tire force (FE) (front)','Interpreter','latex')

ax(4) = subplot(2,2,4);
for jj = 1:length(kf)
    for ii = 1:length(alphaf)
        val1(ii,jj) = Fxr(ii,jj);
        val2(ii,jj) = Fyr(ii,jj);
    end
end
surf(kr,rad2deg(alphar),sqrt(val1.^2 + val2.^2)./Fzr)
ylabel('$\alpha_f$ [deg]','Interpreter','latex'); 
xlabel('$\kappa_f$ [-]','Interpreter','latex'); 
zlabel('$\sqrt{F_{x,f}^2 + F_{y,f}^2}/F_{z,f}$ [-]','Interpreter','latex'); 
title('Resulting tire force (FE) (rear)','Interpreter','latex')

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%% Pacejka Magic Formula with Friction Elipse but inputs are vx and vy
% % ---- model parameterization ----
% params.mux = [1.2 1.2];
% params.Bx = [11.7 11.1];
% params.Cx = [1.69 1.69];
% params.Ex = [0.377 0.362];
% params.muy = [0.935 0.961];
% params.By = [8.86 9.30];
% params.Cy = [1.19 1.19];
% params.Ey = [-1.21 -1.11];
% 
% params.m = 2100;
% params.lf = 1.3; 
% params.lr = 1.5;
% params.g = 9.8;
% % ---- variables ----
% alphaf = linspace(-pi/6,pi/6,51); alphar = alphaf;
% kf = linspace(-1,1,51); kr = kf;
% 
% Fzf = params.m*params.g*params.lr/(params.lf+params.lr);
% Fzr = params.m*params.g*params.lf/(params.lf+params.lr);
% 
% % ---- Test equation ----
% for jj = 1:length(kf)
%     for ii = 1:length(alphaf)
%         [Fxf(ii,jj),Fyf(ii,jj),Fxr(ii,jj),Fyr(ii,jj)] = magic_formula_friction_elipses(params,Fzf,Fzr,alphaf(ii),alphar(ii),kf(jj),kr(jj));
%     end
% end


    