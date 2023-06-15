function [out,misc] = combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,sel)
% Function to determine the total forces and moments acting on the vehcile
% [out,misc] = combined_forces_FXFY(params,vx,vy,r,omegaf,omegar,delta,sel)
% the input to this function are:
% model parameters          --> params
% vehicle velocities        --> vx, vy 
% yaw rate                  --> r
% steering angle            --> delta
% output select             --> sel
% sel is chosen between 1, 2, or 3, and the output either FX, FY, or MZ,
% respectively
% function outputs          --> out (i.e, FX, FY, or MZ)
%                           --> misc (extra information for debugging)

% ---- parameters
m = params.m; % mass
lf = params.lf; % CoM to front wheel center
lr = params.lr; % CoM to rear wheel center
Re = params.Re; % effective tire radius
g = params.g; % acceleration due to gravity 

% ---- velocity of each wheel in its locak cordinate system
vxf = vx.*cos(delta) + sin(delta).*(vy + lf.*r); 
vyf = cos(delta).*(vy + lf.*r) - vx.*sin(delta);
vxr = vx;
vyr = vy + lr.*r;
% ---- lateral slip angle ----
alphaf = -atan(vyf./vxf);
alphar = -atan(vyr./vxr);
% ---- longitudinal slip ----
kappaf = (Re.*omegaf - vxf)./vxf;
kappar = (Re.*omegar - vxr)./vxr;
% ---- maximum normal forces on the front and the real
Fzf = m.*g.*lr/(lf+lr);
Fzr = m.*g.*lf/(lf+lr);
    
% ---- determining the individual forces using the magic formula and
% fiction elipses ----
[~,Fxf,Fyf,Fxr,Fyr] = magic_formula_friction_elipses(params,Fzf,Fzr,alphaf,alphar,kappaf,kappar,100);

% ---- total forces and moments acting on the vehcile ----
switch sel
    case 1
        out = Fxf.*cos(delta) - Fyf.*sin(delta) + Fxr; %%%% FX
    case 2
        out = Fyf.*cos(delta) + Fxf.*sin(delta) + Fyr; %%%% FY
    case 3
        out = lf.*Fyf.*cos(delta) + lf.*Fxf.*sin(delta) - lr.*Fyr; %%%% MZ
    case 4
        out = Fxf; %%%% Fxf
    case 5
        out = Fyf; %%%% Fyf
    case 6
        out = Fxr; %%%% Fxr
    case 7
        out = Fyr; %%%% Fyr
end

%---- misc variables for debugging and plotting
misc.alphaf = alphaf;
misc.alphar = alphar;
misc.kappaf = kappaf;
misc.kappar = kappar;
misc.Fzf = Fzf;
misc.Fzr = Fzr;
misc.Fxf = Fxf;
misc.Fxr = Fxr;
misc.Fyf = Fyf;
misc.Fyr = Fyr;


