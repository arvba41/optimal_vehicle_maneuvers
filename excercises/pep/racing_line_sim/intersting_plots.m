clear all; 
clc;

% initial values
Xps = -7; 
Yps = 0;
Vxs = 0; 
Vys = 50/3.6;

% boundry values
Xpf = -7;
Ypf = 0;

% parameters
params.m = 2000;
params.g = 9.81;
params.mu = 1.2;

% path
R1i = 3;
R2i = 150;
R1o = 9;
R2o = 155;

%% plottings

figno_c = 10;
% clearing plots
figure(figno_c); clf;
figure(figno_c+1); clf;
figure(figno_c+2); clf;
figure(figno_c+3); clf;

for ii = 7:8 % 5:6 % 2:2:4 % 1:2:3 % 
    switch ii
        case 1 
            load("racing_line_full_50m_dbl_harpin.mat");
            lgnd(ii) = "min $t$, nom $I_{zz}$";
            clr = [0 0.4470 0.7410];
        case 2
            load("racing_line_full_70m_dbl_harpin.mat");
            lgnd(ii) = "min $t$, +10\% $I_{zz}$";
            clr = [0 0.4470 0.7410];
        case 3
            load("hrp_50m_30kmph_vf_opt_init_res_v2.mat");
            lgnd(ii) = "min $-v_f$, $I_{zz}$";
            clr = [0.8500 0.3250 0.0980];
        case 4
            load("hrp_50m_30kmph_intertial_inc_10pct_vf_opt_init_res.mat");
            lgnd(ii) = "min $-v_f$, +10\% $I_{zz}$";
            clr = [0.8500 0.3250 0.0980];
        case 5
            load("hrp_50m_30kmph_t_opt_free_xpos.mat");
            lgnd(ii) = "min $-v_f$, free xpos";
            clr = [0 0.4470 0.7410];
        case 6
            load("hrp_50m_30kmph_vf_opt_free_xpos.mat");
            lgnd(ii) = "min $-v_f$, free xpos";
            clr = [0.8500 0.3250 0.0980];    
        case 7
            load("hrpo6_55m_30kmph_t_opt_free_vx_xpos.mat");
            lgnd(ii) = "min $-v_f$, free xpos";
            clr = [0 0.4470 0.7410];
            pow = 6;
        case 8
            load("hrpo6_55m_30kmph_vf_sub_opt_free_vx_xpos.mat");
            lgnd(ii) = "min $-v_f$, free xpos";
            clr = [0.8500 0.3250 0.0980];    
            pow = 6;
    end