

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

clear all; clc;

%%% vehicle paramteres
Xfin = 50; % finishing position 
mass = 1000; % mass of the car
mu = 0.5; % Gs 
g = 9.8; % acceleration due to gravity

%%% obstacle parameters
Xa = Xfin; R1 = 10; R2 = 1.5; Ylim = 1; pow = 2;

%%% optimization limits
Ymax = 5; % upper limit position y

%%% Run case 1 
problem1_uncon;

%%% Run case 2
problem1_sinpathcons_lower;

%%% Run case 3
problem1_sinpathcons_upper;

%%% Run case 4
    %%%%%% special case
        mul = 2; % sine obstacle multiplier
        amp = 1.1; % obstacle multiplier amplitude
        Xfin = 150;
problem1_sinpathcons_upperlower;

%%% Run case 5
problem1_avoidence;



