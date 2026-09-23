%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [CONFIDENTIAL]
% plot reponse curve of physical control systems 
% Transimission time: 8.3 ms
% Control rate: 24 Hz
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xlim_u =[0,Tf]; %[2,3];%[0,Tf];
fig1 = figure;
% set(fig1, 'Position', [0 2200 900 620]);%[x y width height]
ft_size = 25;
line_width = 2;
set(gca, 'FontSize', ft_size);
plot(TIME,L1OUT,'LineWidth',line_width);
hold on;
plot(TIME,L2OUT,'LineWidth',line_width);
plot(TIME,BASINOUT,'LineWidth',line_width);
title('Loop 1');
set(gca,'XLim',[0 Tf]);
set(gca,'Ylim',[0,26]);
xlabel('t/s', 'FontSize',ft_size);
ylabel('L/m', 'FontSize',ft_size);
hold on
plot(0:delta_t:Tf, ones(1, Tf/delta_t+1)*L2sp, 'g--','LineWidth',line_width);
legend('L1','L2','LR','L2*');
set(gca, 'FontSize', ft_size);
% saveas(gca, 'loop1_x_84.eps','epsc');


fig2 = figure;
%  set(fig2, 'Position', [0 750 900 620]);%[x y width height]
subplot(2,1,1)
plot(TIME,U1OUTc,'r','LineWidth',line_width);
hold on;
plot(TIME,U1OUT,'LineWidth',line_width);
title('u1','FontSize',ft_size);
xlabel('t/s', 'FontSize',ft_size);
ylabel('V/v', 'FontSize',ft_size);
legend({'$u1$','$\hat{u1}$'},'Interpreter','latex');
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim_u);
subplot(2,1,2)
plot(TIME,Trigger/system_rate,'LineWidth',line_width)
set(gca,'Ylim',[0,Tf]);
xlabel('t/s', 'FontSize',ft_size);
ylabel('Trigger', 'FontSize',ft_size);
set(gca,'Xlim',xlim_u);
set(gca, 'FontSize', ft_size);
% saveas(gca, 'loop1_u_84.eps','epsc');

fig3 = figure;
% set(fig3, 'Position', [1050 2200 900 620]);%[x y width height]
line_width = 2;
set(gca, 'FontSize', ft_size);
plot(TIME,L1OUT1,'LineWidth',line_width);
hold on;
plot(TIME,L2OUT1,'LineWidth',line_width);
plot(TIME,BASINOUT1,'LineWidth',line_width);
title('Loop2');
set(gca,'Ylim',[0,14]);
set(gca,'XLim',[0 Tf]);
xlabel('t/s', 'FontSize',ft_size);
ylabel('L/m', 'FontSize',ft_size);
hold on
plot(0:delta_t:Tf, ones(1, Tf/delta_t+1)*L2sp2, 'g--','LineWidth',line_width);
legend('L3','L4','LR2','L4*');
set(gca, 'FontSize', ft_size);
% saveas(gca, 'loop2_x_84.eps','epsc');

fig4 = figure;
% set(fig4, 'Position', [1050 750 900 620]);%[x y width height]
subplot(2,1,1)
plot(TIME,U2OUTc,'r','LineWidth',line_width);
hold on;
plot(TIME,U1OUT1,'LineWidth',line_width);
legend({'$u2$','$\hat{u2}$'},'Interpreter','latex');
title('u2','FontSize',ft_size);
xlabel('t/s', 'FontSize',ft_size);
ylabel('V/v', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'XLim',[0 Tf]);
subplot(2,1,2)
plot(TIME,Trigger1/system_rate,'LineWidth',line_width);
xlabel('t/s', 'FontSize',ft_size);
ylabel('Trigger', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'XLim',[0 Tf]);
set(gca,'YLim',[0 Tf]);
% saveas(gca, 'loop2_u_84.eps','epsc');



fig5 = figure;
% set(fig5, 'Position', [2000 2200 900 620]);%[x y width height]
ft_size = 25;
line_width = 2;
set(gca, 'FontSize', ft_size);
plot(TIME,L1OUT2,'LineWidth',line_width);
hold on;
plot(TIME,L2OUT2,'LineWidth',line_width);
plot(TIME,BASINOUT2,'LineWidth',line_width);
title('Loop 3');
set(gca,'XLim',[0 Tf]);
xlabel('t/s', 'FontSize',ft_size);
ylabel('L/m', 'FontSize',ft_size);
hold on
plot(0:delta_t:Tf, ones(1, Tf/delta_t+1)*L2sp, 'g--','LineWidth',line_width);
set(gca,'Ylim',[0,26]);
set(gca,'XLim',[0 Tf]);
legend('L5','L6','LR3','L6*');
set(gca, 'FontSize', ft_size);
% saveas(gca, 'loop3_x_84.eps','epsc');

fig6 = figure;
% set(fig6, 'Position', [2000 750 900 620]);%[x y width height]
subplot(2,1,1)
plot(TIME,U3OUTc,'r','LineWidth',line_width);
hold on;
plot(TIME,U1OUT2,'LineWidth',line_width);
title('u3','FontSize',ft_size);
xlabel('t/s', 'FontSize',ft_size);
ylabel('V/v', 'FontSize',ft_size);
set(gca,'Xlim',xlim_u);
legend({'$u3$','$\hat{u3}$'},'Interpreter','latex');
set(gca, 'FontSize', ft_size);
subplot(2,1,2)
plot(TIME,Trigger2/system_rate,'LineWidth',line_width)
set(gca,'Xlim',xlim_u);
xlabel('t/s', 'FontSize',ft_size);
ylabel('Trigger', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'XLim',[0 Tf]);
set(gca,'YLim',[0 Tf]);
% saveas(gca, 'loop3_u_84.eps','epsc');

fig7 = figure;
% set(fig7, 'Position', [3250 2200 900 620]);%[x y width height]
line_width = 2;
set(gca, 'FontSize', ft_size);
plot(TIME,L1OUT3,'LineWidth',line_width);
hold on;
plot(TIME,L2OUT3,'LineWidth',line_width);
plot(TIME,BASINOUT3,'LineWidth',line_width);
title('Loop4');
set(gca,'XLim',[0 Tf]);
xlabel('t/s', 'FontSize',ft_size);
ylabel('L/m', 'FontSize',ft_size);
hold on
plot(0:delta_t:Tf, ones(1, Tf/delta_t+1)*L2sp2, 'g--','LineWidth',line_width);
set(gca,'Ylim',[0,14]);
set(gca,'XLim',[0 Tf]);
legend('L7','L8','LR3','L8*');
set(gca, 'FontSize', ft_size);
% saveas(gca, 'loop4_x_84.eps','epsc');

fig8 = figure;
% set(fig8, 'Position', [3250 750 900 620]);%[x y width height]
subplot(2,1,1)
plot(TIME,U4OUTc,'r','LineWidth',line_width);
hold on;
plot(TIME,U1OUT3,'LineWidth',line_width);
legend({'$u4$','$\hat{u4}$'},'Interpreter','latex');
set(gca,'XLim',[0 Tf]);
title('u4','FontSize',ft_size);
xlabel('t/s', 'FontSize',ft_size);
ylabel('V/v', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
subplot(2,1,2)
plot(TIME,Trigger3/system_rate,'LineWidth',line_width);
xlabel('t/s', 'FontSize',ft_size);
ylabel('Trigger', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'XLim',[0 Tf]);
set(gca,'YLim',[0 Tf]);
% saveas(gca, 'loop4_u_84.eps','epsc');

