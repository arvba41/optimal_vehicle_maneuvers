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
%                 - fl_prtcl_obsavoid_elp.m
%                 - plottingrest_fric.m
%                 - rl_prtcl_obsavoid_elp.m
%                 - plottingrest_rate.m
%                 - plottingrest_report_fep.m (only for personal use)
%                 - plottingrest_report_fep_detailed.m (only for personal use)
%
% References: Optimal Vehicular Menuvers Course material in git (private)
%
% Disclaimer: This script is provided "as is" without warranty of any kind, either express or implied.
%             The author assumes no liability for any errors or omissions, or for any damages
%             resulting from the use of this script.
clear all; clc

%% paramters 
% ---- model ----
mdl.Xfin = 50;      % finishing position x
mdl.mass = 500;     % mass of the car
mdl.mu = 0.8;       % friction
mdl.g = 9.8;        % acceleration due to gravity
% ---- constraints ----
cons.Ymax = 5;              % upper limit for the y direction 
cons.delta_max = pi/2;      % angle limit
cons.ddelta_max = pi/6;     % angle rate limit
cons.vxinit = 40/3.6;      % intial vx speed
% ---- obstacle ----
obst.Xa = mdl.Xfin; % center of the elipse
obst.R1 = 4;        % radius of elipse in x direction
obst.R2 = 2;      % radius of elipse in x direction
obst.pow = 6;       % curvature of elipse 
% ---- optimization options ----
optops.N = 40;        % number of control intervals for the optimization

%% Friction-limited Particle with Obstacle Avoidance:
%
% In this variation of the friction-limited particle model, we introduce an obstacle centered at
% a specific position Xa. The goal of the particle is to reach a target position while avoiding
% the obstacle.

% ---- optimization ----
sol = fl_prtcl_obsavoid_elp(mdl,obst,cons,optops,[]);

% ---- result storate ----
optres_fl.topt = sol.topt; % optimal time
optres_fl.x = sol.x; optres_fl.y = sol.y; % positions
optres_fl.vx = sol.vx; optres_fl.vy = sol.vy; % velocities
optres_fl.ux = sol.ux; optres_fl.uy = sol.uy; % forces
optres_fl.tvec = sol.tvec; % time vector 

% ---- plotting the results ----
figure(10); clf; tiledlayout(3,2);
ax = plottingrest_fric(optres_fl,'fric-lmt');
optres1 = optres_fl; % for the report


%% Rate-limited Particle with Obstacle Avoidance:
%
% In this variation of the rate-limited particle model, we introduce an obstacle centered at
% a specific position Xa. The goal of the particle is to reach a target position while avoiding
% the obstacle.

% ---- optimization ----
sol = rl_prtcl_obsavoid_elp(mdl,obst,cons,optops);

% ---- result storate ----
optres_rl.topt = sol.topt; % optimal time
optres_rl.x = sol.x; optres_rl.y = sol.y; % positions
optres_rl.vx = sol.vx; optres_rl.vy = sol.vy; % velocities
optres_rl.ux = sol.ux; optres_rl.uy = sol.uy; % forces
optres_rl.u = sol.u;
optres_rl.delta = sol.delta; optres_rl.ddelta = sol.ddelta;
optres_rl.tvec = sol.tvec; % time vector 
optres2 = optres_rl; % for the report

% ---- plotting the results ----
% figure(10); clf; tiledlayout(3,2);
ax = plottingrest_rate(optres_rl,'rate-lmt');

% ---- obstacle area plot ----
elps = @(x,y)-((x-obst.Xa)/obst.R1).^obst.pow-(y/obst.R2).^obst.pow+1; % obstacle equation 
xrange = [0 100]; yrange = [0 3]; 
fimplicit(ax(1),@(x,y) elps(x,y),[0 100 0 5],'--','Color','k');
ylim(ax(1),[-0.5 3.5]); 
% yline(ax(1),3,'LineStyle','--','Color','k');
% yline(ax(1),0,'LineStyle','--','Color','k');

% Get latex font in ticks
h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

%% report ploting (only for personla use)
plottingrest_report_fep(optres_fl,optres_rl,obst);

optres_fl.int_idx = find(round(elps(optres_fl.x,optres_fl.y)) == 0);
optres_rl.int_idx = find(round(elps(optres_rl.x,optres_rl.y)) == 0);
plottingrest_report_fep_detailed(optres_fl,optres_rl,mdl,cons);

