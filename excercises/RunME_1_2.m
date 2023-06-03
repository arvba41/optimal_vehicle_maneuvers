clear all; clc;

%% Car Avoiding an obstacle
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP
% 
% 4 different cases have been presented here 
% Case 1: No obstacle 
% Case 2: sinusoidal path with lower bound
% Case 3: sinusoidal path with upper bound
% Case 4: constained trajectory 
% Case 5: eliptical obstacle at distance 

%%% vehicle paramteres
Xfin = 50; % finishing position 
mass = 1000; % mass of the car
mu = 0.5; % Gs 
g = 9.8; % acceleration due to gravity

delta_max = g; % acceleration in y
ddelta_max = 1; % jerk in y

%%% obstacle parameters
Xa = Xfin; R1 = 10; R2 = 1.5; Ylim = 1; pow = 2;

%%% optimization limits
Ymax = 5; % upper limit position y

%%% Paths for the different models
path1 = "problem1\"; %% path for point model
path2 = "problem2\"; %% path for kinemetic model

%%% Run case 1 
% problem 1
file = "problem1_uncon";
tStart = tic;           
run(char(path1 + file));
tEnd(1,1) = toc(tStart);
% output recording 
prob{1}.topt = prob1.topt; % optimal time
prob{1}.x = prob1.x; prob{1}.y = prob1.y; % outputs
prob{1}.ux = prob1.ux; prob{1}.uy = prob1.uy; % forces
prob{1}.tvec = prob1.tvec; % time vector 

% problem 2
file = "problem2_uncon";
tStart = tic;           
run(char(path2 + file));
tEnd(1,2) = toc(tStart);
% output recording 
prob{2}.topt = prob1.topt; % optimal time
prob{2}.x = prob1.x; prob{2}.y = prob1.y; % outputs
prob{2}.ux = prob1.ux; prob{2}.uy = prob1.uy; % forces
prob{2}.tvec = prob1.tvec; % time vector 
prob{2}.cons.Flim = mass*mu*g; 

plotting1(prob);

%% Run case 4
Xfin = 150; % finishing position 

mul = 3; % obstacle multiplier frequency
amp = 2; % obstacle multiplier amplitude

N = 40; % number of control intervals

% problem 1
file = "problem1_sinpathcons_upperlower";
tStart = tic;           
run(char(path1 + file));
tEnd(1,1) = toc(tStart);
% output recording 
prob{1}.topt = prob4.topt; % optimal time
prob{1}.x = prob4.x; prob{1}.y = prob4.y; % outputs
prob{1}.ux = prob4.ux; prob{1}.uy = prob4.uy; % forces
prob{1}.tvec = prob4.tvec; % time vector 

% problem 2
file = "problem2_sinpathcons_upperlower";
tStart = tic;           
run(char(path2 + file));
tEnd(1,2) = toc(tStart);
% output recording 
prob{2}.topt = prob4.topt; % optimal time
prob{2}.x = prob4.x; prob{2}.y = prob4.y; % outputs
prob{2}.ux = prob4.ux; prob{2}.uy = prob4.uy; % forces
prob{2}.tvec = prob4.tvec; % time vector 
prob{2}.cons.Flim = mass*mu*g; 

prob{2}.cons.yllim = y1curve(prob4.x); 
prob{2}.cons.yulim = yucurve(prob4.x);

plotting2(prob);

%% Run case 5
Xfin = 50; % finishing position 

% mul = 3; % obstacle multiplier frequency
% amp = 2; % obstacle multiplier amplitude

N = 40; % number of control intervals

Xa = Xfin; R1 = 5; R2 = 1.3; pow = 2;

Ymax = 5; % upper limit 

% problem 1
file = "problem1_avoidence";
tStart = tic;           
run(char(path1 + file));
tEnd(1,1) = toc(tStart);
% output recording 
prob{1}.topt = prob5.topt; % optimal time
prob{1}.x = prob5.x; prob{1}.y = prob5.y; % outputs
prob{1}.ux = prob5.ux; prob{1}.uy = prob5.uy; % forces
prob{1}.tvec = prob5.tvec; % time vector 

% problem 2
file = "problem2_avoidence";
tStart = tic;           
run(char(path2 + file));
tEnd(1,2) = toc(tStart);
% output recording 
prob{2}.topt = prob5.topt; % optimal time
prob{2}.x = prob5.x; prob{2}.y = prob5.y; % outputs
prob{2}.ux = prob5.ux; prob{2}.uy = prob5.uy; % forces
prob{2}.tvec = prob5.tvec; % time vector 
prob{2}.cons.Flim = mass*mu*g; 
prob{2}.cons.obst = obst(prob5.x,prob5.y);
prob{2}.cons.yobst = yobst(prob5.x);


plotting3(prob);

%%
% %%% Run case 2
% problem1_sinpathcons_lower;
% 
% %%% Run case 3
% problem1_sinpathcons_upper;
% 
% %%% Run case 4
%     %%%%%% special case
%         mul = 2; % sine obstacle multiplier
%         amp = 1.1; % obstacle multiplier amplitude
%         Xfin = 150;
% problem1_sinpathcons_upperlower;
% 
% %%% Run case 5
% problem1_avoidence;
% 
% 
% 
% % 