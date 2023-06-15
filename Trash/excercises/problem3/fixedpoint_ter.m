clc; clear;

global V2 V1 I1 I2

I1 = 0.3; V1 = 0.3;
I2 = 2.5; V2 = 0.7;

figure(1); clf;

tiledlayout('flow');

nexttile;
kvec = linspace(0,5,1000);
plot(kvec,ksolve0(kvec),'LineWidth',2);
ylabel('$f_o(x)$','Interpreter','latex');
xlabel('$x$','Interpreter','latex');
box off;
yline(0,'LineWidth',2,'Color','k','LineStyle','--'); hold on;

h = findall(gcf,'Type','axes'); % An array if you have subplots
set(h, 'TickLabelInterpreter', 'latex')

k = 5;
for ii = 1:100
   k = ksolve(k);
   plot(k,ksolve0(k),'x','MarkerSize',20,'LineWidth',2);
end

Is(1) = I1/(exp(k*V1)-1);
Is(2) = I2/(exp(k*V2)-1);
Is = mean(Is);

figure(2); clf;
Vd = 0:0.01:1;
Id = @(x) Is*(exp(k*Vd)-1);

semilogy(Vd,abs(Id(Vd)),'LineWidth',2); grid on; hold on
ylabel('$I_d$ [A]','Interpreter','latex');
xlabel('$V_d$ [V]','Interpreter','latex');

figure(1)
k = 0.1;
for ii = 1:100
   k = ksolve(k);
   plot(k,ksolve0(k),'x','MarkerSize',20,'LineWidth',2);
end

Is(1) = I1/(exp(k*V1)-1);
Is(2) = I2/(exp(k*V2)-1);
Is = mean(Is);

figure(2); 
% Vd = :0.01:1;
Id = @(x) Is*(exp(k*Vd)-1);

semilogy(Vd,Id(Vd),'LineWidth',2); grid on; hold off;
ylabel('$I_d$ [A]','Interpreter','latex');
xlabel('$V_d$ [V]','Interpreter','latex');
legend('k high','k low','Interpreter','latex');

function val = ksolve(k)
    global V2 V1 I1 I2
    val = 1/V2*log(1 + I2/I1*(exp(V1*k)-1));
end

function val = ksolve0(k)
    global V2 V1 I1 I2
    val = k - (1/V2*log(1 + I2/I1*(exp(V1*k)-1)));
end