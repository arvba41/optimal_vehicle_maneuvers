%% Diode Equation
% using U0=1/k;
diodeE=@(Io,Uo,U) Io*(exp(U/Uo)-1);

%% Data Points
% SB 160
V1=0.3;I1=1e-2;
V2=0.75;I2=3;
%% SB 150
V2=0.18;I2=1e-2;
V1=0.68;I1=3;

%% diode Solve for k Equation.
diodeFixPS =@(k) log(1+I1/I2*(exp(k*V2)-1))/V1;
diodeDFix = @(k) V2/V1/(1+exp(-k*V2)*(1-I1/I2));

diodeFixP =@(V1,V2,I1,I2,k) log(1+I1/I2*(exp(k*)-1))/V1;
%% Equation solving and fixed point iterations
figure(1);clf
k=logspace(-3,log10(14),100);
plot(k,k-diodeFixPS(k))
hold on
grid on
xlabel('k')
title('Two solutions')
k=8;
disp('Starting Iterations')
for ii=1:10,
    plot(k,k-diodeFixPS(k),'x');
    disp(['k = ', num2str(k)])
    k=diodeFixPS(k);
end
disp(['k = ', num2str(k)])