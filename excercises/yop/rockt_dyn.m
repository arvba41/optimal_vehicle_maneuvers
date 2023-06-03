function [x_dot,y] = rockt_dyn(x,u,params)

% ---- parameters ----
D0  = params.D0;
g0 = params.g0;
beta = params.beta;
r0 = params.r0;
c = params.c;

% ---- inputs 
F = u;

% ---- states ----
v = x(1);
h = x(2);
m = x(3);

% ---- functions ----
D = D0*v^2*exp(-beta*h); % function for drag
g = g0*(r0/(r0 + h))^2; % function for gravitation acceleration

% --- ODEs ----
v_dot = (u*c - D)/m - g;
h_dot = v;
m_dot = -F;

x_dot = [v_dot; h_dot; m_dot];
y = 0;
end