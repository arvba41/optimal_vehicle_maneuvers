% This MATLAB script investigates two different types of particle motion:
% the friction-limited particle and the friction-limited particle with 
% rate-limited direction control.
%
% The criterian for optimization is the minimum time to reach the target.
%
% Additionally, particles are simulated using a simpler version of the
% rate-limited particle model.
% 
% This script outputs graphs comparing the positions and forces of the two 
% types of particle models, as well as their time to reach the target.
%
% Author: Arvind Balachandran
% Date: 2023-05-10
% Version: 1.0
%
% Usage: Run this script in MATLAB and view the generated graphs.
%        Adjust the initial conditions as desired to investigate different scenarios.
%
% Required files: This script requires the following files to be in the same directory:
%                 - [file1].m
%                 - [file2].m
%                 - [file3].m
%
% References: Optimal Vehicular Menuvers Course material in git (private)
%
% Disclaimer: This script is provided "as is" without warranty of any kind, either express or implied.
%             The author assumes no liability for any errors or omissions, or for any damages
%             resulting from the use of this script.
clear all; clc

%% paramters 
% ---- model ----
mdl.Xfin = 23;      % finishing position x
mdl.mass = 1;     % mass of the car
mdl.mu = 0.5;       % friction
mdl.g = 9.8;        % acceleration due to gravity
% ---- constraints ----
cons.Ymax = 8;              % upper limit for the y direction 
cons.delta_max = pi/6;  % acceleration in y
cons.ddelta_max = pi/4;     % jerk in y
cons.vxinit = 55/3.6;      % intial vx speed
% ---- obstacle ----
obst.Xa = mdl.Xfin; % center of the elipse
obst.R1 = 2;        % radius of elipse in x direction
obst.R2 = 3.2;      % radius of elipse in x direction
obst.pow = 6;       % curvature of elipse 
% ---- optimization options ----
optops.N = 100;        % number of control intervals for the optimization

% % % % %% Friction-limited Particle with Obstacle Avoidance:
% % % % %
% % % % % In this variation of the friction-limited particle model, we introduce an obstacle centered at
% % % % % a specific position Xa. The goal of the particle is to reach a target position while avoiding
% % % % % the obstacle.
% % % % 
% % % % global opti
% % % % 
% % % % % ---- optimization ----
% % % % sol = fl_prtcl_obsavoid_elp_lim(mdl,obst,cons,optops);
% % % % 
% % % % % ---- result storate ----
% % % % optres.topt = sol.topt; % optimal time
% % % % optres.x = sol.x; optres.y = sol.y; % positions
% % % % optres.vx = sol.vx; optres.vy = sol.vy; % velocities
% % % % optres.ux = sol.ux; optres.uy = sol.uy; % forces
% % % % optres.tvec = sol.tvec; % time vector 
% % % % 
% % % % %%
% % % % % ---- plotting the results ----
% % % % figure(10); clf; tiledlayout(3,2);
% % % % ax = plottingrest(optres,'fric-lmt');
% % % % optres1 = optres; % for the report
% % % % 
% % % % % ---- obstacle area plot ----
% % % % elps = @(x,y)-((x-obst.Xa)/obst.R1).^obst.pow-(y/obst.R2).^obst.pow+1; % obstacle equation 
% % % % % xrange = [0 100]; yrange = [0 3]; 
% % % % fimplicit(ax(1),@(x,y) elps(x,y),[0 mdl.Xfin 0 4],'--','Color','k');
% % % % % ylim(ax(1),[-0.5 3.5]); 
% % % % % yline(ax(1),3,'LineStyle','--','Color','k');
% % % % % yline(ax(1),0,'LineStyle','--','Color','k');

%% Rate-limited Particle with Obstacle Avoidance:
%
% In this variation of the rate-limited particle model, we introduce an obstacle centered at
% a specific position Xa. The goal of the particle is to reach a target position while avoiding
% the obstacle.

% ---- optimization ----
sol = rl_prtcl_obsavoid_elp_lim(mdl,obst,cons,optops);

% ---- result storate ----
optres.topt = sol.topt; % optimal time
optres.x = sol.x; optres.y = sol.y; % positions
optres.vx = sol.vx; optres.vy = sol.vy; % velocities
optres.ux = sol.ux; optres.uy = sol.uy; % forces
optres.delta = sol.delta;
optres.tvec = sol.tvec; % time vector 
optres2 = optres; % for the report

%%
% ---- plotting the results ----
figure(10); clf
% ax = plottingrest(optres,'rate-lmt');
tiledlayout('flow'); clf;

ax(1) = nexttile([1 2]);
plot(optres.x,optres.y); hold on
ylabel('$y$ [m]','Interpreter','latex');
xlabel('$x$ [m]','Interpreter','latex');
elps = @(x,y)-((x-obst.Xa)/obst.R1).^obst.pow-(y/obst.R2).^obst.pow+1; % obstacle equation 
fimplicit(ax(1),@(x,y) elps(x,y),[0 mdl.Xfin 0 4],'--','Color','k'); hold off;

ax(2) = nexttile;
plot(optres.tvec,optres.vx*3.6); hold on
ylabel('$v_y$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

ax(3) = nexttile;
plot(optres.tvec,optres.vy*3.6); hold on
ylabel('$v_y$ [km/h]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

ax(4) = nexttile;
yyaxis left
plot(optres.tvec(2:end),[optres.ux.*cos(optres.delta(2:end)); optres.ux.*sin(optres.delta(2:end))]); hold on
ylabel('$a_{x,y}$ [m/s$^2$]','Interpreter','latex');
yyaxis right
plot(optres.tvec(2:end),optres.ux); hold on
xlabel('$t$ [s]','Interpreter','latex');
ylabel('$a$ [m/s$^2$]','Interpreter','latex');
yline(mdl.mu*mdl.g,'--');
yline(-mdl.mu*mdl.g,'--');

ax(5) = nexttile;
plot(optres.tvec(2:end),optres.uy); hold on
ylabel('$\delta$ [m]','Interpreter','latex');
xlabel('$t$ [s]','Interpreter','latex');

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%%
% ---- obstacle area plot ----

ylim(ax(1),[-0.5 3.5]); 
yline(ax(1),3,'LineWidth',2,'LineStyle','--','Color','k');
yline(ax(1),0,'LineWidth',2,'LineStyle','--','Color','k');

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%% report ploting
plottingrest_report_fep(optres1,optres2);