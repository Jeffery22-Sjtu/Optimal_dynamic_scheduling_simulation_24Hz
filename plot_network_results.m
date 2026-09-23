%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [CONFIDENTIAL]
% plot network resource allocation and packet reception
% Transimission time: 8.3 ms
% Control rate: 24 Hz
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

line_width = 1;
xlim=[0,Tf];
ylim=[0,1.1];

fig9 = figure;
% set(fig9, 'Position', [8 8 900 620]);%[x y width height]
subplot(3,1,1)
plot(TIME,Trigger/system_rate,'LineWidth',line_width);
hold on;
ylabel('Trigger', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',xlim);
subplot(3,1,2)
plot(TIME,access1,'LineWidth',line_width)
ylabel('Sending', 'FontSize',ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',ylim);
set(gca, 'FontSize', ft_size);
subplot(3,1,3)
plot(TIME,Net_OUT3,'LineWidth',line_width)
xlabel('t/s', 'FontSize',ft_size);
ylabel('Receiving', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',ylim);
% saveas(gca, 'loop1_net.eps','epsc');

fig10 = figure;
% set(fig10, 'Position', [1050 8 900 620]);%[x y width height]
subplot(3,1,1)
plot(TIME,Trigger1/system_rate,'LineWidth',line_width);
hold on;
ylabel('Trigger', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',xlim);
subplot(3,1,2)
plot(TIME,access2,'LineWidth',line_width)
ylabel('Sending', 'FontSize',ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',ylim);
set(gca, 'FontSize', ft_size);
subplot(3,1,3)
plot(TIME,Net_OUT4,'LineWidth',line_width)
xlabel('t/s', 'FontSize',ft_size);
ylabel('Receiving', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',ylim);
% saveas(gca, 'loop2_net.eps','epsc');

fig11 = figure;
% set(fig11, 'Position', [2000 8 900 620]);%[x y width height]
subplot(3,1,1)
plot(TIME,Trigger2/system_rate,'LineWidth',line_width);
hold on;
ylabel('Trigger', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',xlim);
subplot(3,1,2)
plot(TIME,access3,'LineWidth',line_width)
ylabel('Sending', 'FontSize',ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',ylim);
set(gca, 'FontSize', ft_size);
subplot(3,1,3)
plot(TIME,Net_OUT6,'LineWidth',line_width)
xlabel('t/s', 'FontSize',ft_size);
ylabel('Receiving', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',ylim);
% saveas(gca, 'loop3_net.eps','epsc');

fig12 = figure;
% set(fig12, 'Position', [3250 8 900 620]);%[x y width height]
subplot(3,1,1)
plot(TIME,Trigger3/system_rate,'LineWidth',line_width);
hold on;
ylabel('Trigger', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',xlim);
subplot(3,1,2)
plot(TIME,access4,'LineWidth',line_width)
ylabel('Sending', 'FontSize',ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',ylim);
set(gca, 'FontSize', ft_size);
subplot(3,1,3)
plot(TIME,Net_OUT8,'LineWidth',line_width)
xlabel('t/s', 'FontSize',ft_size);
ylabel('Receiving', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',ylim);
% saveas(gca, 'loop4_net.eps','epsc');
