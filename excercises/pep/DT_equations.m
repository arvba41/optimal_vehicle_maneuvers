syms lyi lxi delta Fxi Fyi vx vy r

sol = [1 0; 0 1; -lyi lxi]*[cos(delta) -sin(delta); sin(delta) cos(delta)]*[Fxi; Fyi]

sol = [cos(delta) sin(delta); -sin(delta) cos(delta)]*([vx; vy] + r*[-lyi; lxi])

% params.a = 1;
% params.b = 2;
% 
% u = [0; 2];
% 
% f = @(t,x) vecfun(x,u,params);
% 
% function xdot = vecfun(x,u,params)
%     x1 = x(1);
%     x2 = x(2);
%     
%     T1 = params.a;
%     T2 = params.b;
%     
%     xdot(1) = (x1 - u)/T1;
%     xdot(1) = (x2 - u)/T2;
% end

test(pi/4,10,1,0.4,0.3,0.1)

function val = test(delta,vx,vy,lyi,lxi,r)
val = [cos(delta) sin(delta); -sin(delta) cos(delta)]*([vx; vy] + r*[-lyi; lxi]);
end