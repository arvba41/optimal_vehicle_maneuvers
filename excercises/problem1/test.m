clear all; clc;

Xa = 10; R1 = 1; R2 = 10; Ya = 10;
xvec = linspace(0,200,500);
yvec = linspace(0,5,500);

figure(1); clf

ax(1) = subplot(221);
for ii = Xa:-1:1
    plot(elipse_fun(xvec,yvec,ii,Ya,R1,R2),...
        'DisplayName',char("Xa = " + num2str(ii))); 
    hold on;
end
hold off; legend('NumColumns',2,'Interpreter','latex');
ylim([0 yvec(end)]); xlim([0 xvec(end)])

ax(3) = subplot(223);
for ii = R1:1:10
    plot(elipse_fun(xvec,yvec,Xa,Ya,ii,R2),...
        'DisplayName',char("R1 = " + num2str(ii))); 
    hold on;
end
hold off; legend('NumColumns',2,'Interpreter','latex');
ylim([0 yvec(end)]); xlim([0 xvec(end)])

ax(3) = subplot(224);
for ii = R2:-1:1
    plot(elipse_fun(xvec,yvec,Xa,Ya,R1,ii),...
        'DisplayName',char("R2 = " + num2str(ii))); 
    hold on;
end
hold off; legend('NumColumns',2,'Interpreter','latex');
ylim([0 yvec(end)]); xlim([0 xvec(end)])

ax(4) = subplot(222);
for ii = 1:Ya
    plot(elipse_fun(xvec,yvec,Xa,ii,R1,R2),...
        'DisplayName',char("Ya = " + num2str(ii))); 
    hold on;
end
hold off; legend('NumColumns',2,'Interpreter','latex');
ylim([0 yvec(end)]); xlim([0 xvec(end)])

linkaxes(ax,'xy');

%% final
figure(2); clf;

Xa = 50; Ylim = 1; pow = 6;  R1 = 10; R2 = 2;

xvec = linspace(0,100,1000);
yvec = linspace(0,5,1000);

elp = @(x,y) -((x-Xa)/R1).^pow -(y/R2).^pow + Ylim;

p = plot(elp(xvec,yvec)); 
% ylim([0 inf]); xlim([0 xvec(end)]);
% % p.XData = xvec; p.YData = yvec;

% %% how does y look like for the elipse

% Xa = 50; Ylim = 1; pow = 3;
% 
% y_elp = @(x) (Ylim - ((x-Xa)/R1).^pow).^(1/pow)*R2;
% 
% p = plot(xvec,y_elp(xvec)); 