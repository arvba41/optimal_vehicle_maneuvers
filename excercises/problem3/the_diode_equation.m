clear all; clc;
Iact = [0.01 0.14 0.5 1.5 3];
Vd = [0.3 0.4 0.5 0.6 0.7];
%%% interested variables
% x(1) --> Is
% x(2) --> Vd


% Io = 2e-4*(exp(12*Vd) - 1);
myfittype = fittype('A*(exp(B*Vd) - 1)',...
    'dependent',{'Iact'},'independent',{'Vd'},...
    'coefficients',{'A','B'})

myfit = fit(Vd',Iact',myfittype)


%% 
clear all; clc

syms I1 I2 x V1 V2

eqn = I1/I2 == (exp(x*V1)-1)/(exp(x*V2)-1)

S = solve(eqn,x)
