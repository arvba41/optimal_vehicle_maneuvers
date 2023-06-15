clc; clear all;

params.D0 = 0.01227;
params.g0 = 9.81;
params.beta = 0.145e-3;
params.r0 = 6.371e6;
params.c = 2060;

[xdot, ~] = rockt_dyn([0; 0; 214.839],0,params)
