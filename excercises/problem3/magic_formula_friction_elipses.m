function [out,Fxf,Fyf,Fxr,Fyr] = magic_formula_friction_elipses(params,Fzf,Fzr,alphaf,alphar,kf,kr,sel)
% Function for the Pacejka Magic Formula with Friction Elipse 
% [Fxf,Fyf,Fxr,Fyr] = magic_formula_friction_elipses(params,vx,vy,r,omegaf,omegar,d)
% the input to this function are:
% model parameters          --> params
% the vertical tire forces  --> Fzf, Fzr 
% slip angle                --> alphaf, alphar
% linear slip               --> kf, kr
% function outputs          --> Fxf, Fyf, Fxr, Fyr

% ---- paramteters ----
muxf = params.mux(1); muxr = params.mux(2); 
muyf = params.muy(2); muyr = params.muy(2);
Cxf = params.Cx(1); Cxr = params.Cx(2);
Cyf = params.Cy(1); Cyr = params.Cy(2);
Bxf = params.Bx(1); Bxr = params.Bx(2);
Byf = params.By(1); Byr = params.By(2);
Exf = params.Ex(1); Exr = params.Ex(2);
Eyf = params.Ey(1); Eyr = params.Ey(2);

% ---- intermediate equations ----
Dxf = muxf.*Fzf; Dxr = muxr.*Fzr;
Dyf = muyf.*Fzf; Dyr = muyr.*Fzr;
% ---- Pacejka’s Magic Formula ----
Fx0f = Dxf.*sin(Cxf.*atan(Bxf.*kf-Exf.*(Bxf.*kf - atan(Bxf.*kf))));
Fx0r = Dxr.*sin(Cxr.*atan(Bxr.*kr-Exr.*(Bxr.*kr - atan(Bxr.*kr))));
Fy0f = Dyf.*sin(Cyf.*atan(Byf.*alphaf-Eyf.*(Byf.*alphaf - atan(Byf.*alphaf))));
Fy0r = Dyr.*sin(Cyr.*atan(Byr.*alphar-Eyr.*(Byr.*alphar - atan(Byr.*alphar))));
% ---- friction elipse ----
Fx0fmax = muxf.*Fzf; % maximum achievable longitudinal tire forces (front)
Fx0rmax = muxr.*Fzf; % maximum achievable longitudinal tire forces (rear)
% Fy0fmax = muyf.*Fzf; % maximum achievable lateral tire forces (front)
% Fy0rmax = muyr.*Fzf; % maximum achievable lateral tire forces (rear)

Fxf = Fx0f; % combined longitudinal force on tire (front)
Fyf = Fy0f.*sqrt(1-(Fx0f/Fx0fmax).^2); % combined lateral force on tire (front)
Fxr = Fx0r; % combined longitudinal force on tire (rear)
Fyr = Fy0r.*sqrt(1-(Fx0r/Fx0rmax).^2); % combined lateral force on tire (rear)

switch sel
    case 1
        out = Fxf; % combined longitudinal force on tire (front)
    case 2
        out = Fyf; % combined lateral force on tire (front)
    case 3
        out = Fxr; % combined longitudinal force on tire (rear)
    case 4
        out = Fyr; % combined lateral force on tire (rear)
    otherwise
        out= []; % do nothing
end


