% function [F_X,F_Y,M_Z] = magic_formula_friction_elipse(params,vx,vy,r,omegaf,omegar,d)
% ---- intermediate equations ----
% ---- maximum normal forces on the front and the real
    Fzf = m*g*lr/(lf+lr);
    Fzr = m*g*lf/(lf+lr);
%%% Wheel and tire modeling
% ---- the velocity of each wheel in its local coordinate system
    vxf = @(d,vx,vy,r) vx*cos(d) + (vy + r*lf)*sin(d);
    vxr = @(d,vx,vy,r) vx;
    vyf = @(d,vx,vy,r) -vx*sin(d) + (vy + r*lf)*cos(d);
    vyr = @(d,vx,vy,r) vy + r*lr;
% ---- slip angle ----
    alphaf = @(vyf,vxf) -atan(vyf/vxf);
    alphar = @(vyr,vxr) -atan(vyr/vxr);
%%--%% Pacejka Magic Formula with Friction Elipse parameters
% ---- model parameterization ----
    muxf = params.mux(1); muxr = params.mux(2); 
    muyf = params.muy(2); muyr = params.muy(2);
    Cxf = params.Cx(1); Cxr = params.Cx(2);
    Cyf = params.Cy(1); Cyr = params.Cy(2);
    Bxf = params.Bx(1); Bxr = params.Bx(2);
    Byf = params.By(1); Byr = params.By(2);
    Exf = params.Ex(1); Exr = params.Ex(2);
    Eyf = params.Ey(1); Eyr = params.Ey(2);
% ---- Front tire longitudinal
    Dxf = muxf*Fzf;
    Fxf = @(kappaf) Dxf*sin(Cxf*atan(Bxf*kappaf-Exf*(Bxf*kappaf-atan(Bxf*kappaf)))); % Resulting tire force (front)
% ---- Rear tire longitudinal
    Dxr = muxr*Fzr;
    Fxr = @(kappar) Dxr*sin(Cxr*atan(Bxr*kappar-Exr*(Bxr*kappar-atan(Bxr*kappar)))); % Resulting tire force (rear)
% ---- Front tire lateral
    Dyf = muyf*Fzf;
    Fy0f = @(alphaf) Dyf*sin(Cyf*atan(Byf*alphaf-Eyf*(Byf*alphaf - atan(Byf*alphaf))));
    Fyf = @(alphaf,kappaf) Fy0f(alphaf)*sqrt(1-(Fxf(kappaf)/Dxf)^2); % combined lateral force on tire (front)
% ---- Rear tire lateral
    Dyr = muyr*Fzr;
    Fy0r = @(alphar) Dyr*sin(Cyr*atan(Byr*alphar-Eyr*(Byr*alphar - atan(Byr*alphar))));
    Fyr = @(alphar,kappar) Fy0r(alphar)*sqrt(1-(Fxr(kappar)/Dxr)^2); % combined lateral force on tire (rear)
%%% Chassis modeling
%%%%% Single track model (ST model)
% ---- The total forces and moment acting on the vehicle center of mass 
    FX = @(vx,vy,r,omegaf,omegar,d) ...
        Fxf(kappaf(omegaf,vxf(d,vx,vy,r)))*cos(d) ...
        - Fyf(alphaf(vyf(d,vx,vy,r),vxf(d,vx,vy,r)),kappaf(omegaf,vxf(d,vx,vy,r)))*sin(d) ...
        + Fxr(kappar(omegar,vxr(d,vx,vy,r))); 
    
    FY = @(vx,vy,r,omegaf,omegar,d) ...
        Fyf(alphaf(vyf(d,vx,vy,r),vxf(d,vx,vy,r)),kappaf(omegaf,vxf(d,vx,vy,r)))*cos(d) ...
        + Fxf(kappaf(omegaf,vxf(d,vx,vy,r)))*sin(d) ...
        + Fyr(alphar(vyr(d,vx,vy,r),vxr(d,vx,vy,r)),kappar(omegar,vxr(d,vx,vy,r))); 
    
    MZ = @(vx,vy,r,omegaf,omegar,d) ...
        lf*Fyf(alphaf(vyf(d,vx,vy,r),vxf(d,vx,vy,r)),kappaf(omegaf,vxf(d,vx,vy,r)))*cos(d) ...
        + lf*Fxf(kappaf(omegaf,vxf(d,vx,vy,r)))*sin(d) ...
        - lr*Fyr(alphar(vyr(d,vx,vy,r),vxr(d,vx,vy,r)),kappar(omegar,vxr(d,vx,vy,r))); 
    
% F_X = FX(vx,vy,r,omegaf,omegar,d);
% F_Y = FY(vx,vy,r,omegaf,omegar,d);
% M_Z = MZ(vx,vy,r,omegaf,omegar,d);

    