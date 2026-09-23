%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [CONFIDENTIAL]
% plot network resource allocation 
% Transimission time: 8.3 ms
% Control rate: 24 Hz
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fig5 = figure;
ft_size = 25;
line_width = 2;
set(fig5, 'Position', [0 8 1200 620]);%[x y width height]
xlim=[0,5];
% xlim=[0,6];

subplot(2,1,1)
area(TIME,access1,'FaceColor','b')
hold on;
area(TIME,access2,'FaceColor','r')
hold on;
area(TIME,access3,'FaceColor','g')
hold on;
area(TIME,access4,'FaceColor','k')
ylabel('Sending', 'FontSize',ft_size);
set(gca,'Xlim',xlim);
set(gca,'Ylim',[0 1.1]);
set(gca, 'FontSize', ft_size);

slot_length = (1/24)/5;
for tt = floor(0*24)/24:(1/24):floor(Tf*24)/24
    for i=1:6
    rectangle('Position',[tt+i*slot_length 0 slot_length 1])
    hold on;
    end
    for i=0:0
    beacon=rectangle('Position',[tt+i*slot_length 0 slot_length 1],'FaceColor','y','LineStyle','none');
    hold on;
    end
end


subplot(2,1,2)
plot(TIME,Net_OUT3,'LineWidth',line_width)
hold on;
plot(TIME,Net_OUT4,'r','LineWidth',line_width)
hold on;
plot(TIME,Net_OUT6,'g','LineWidth',line_width)
hold on;
plot(TIME,Net_OUT8,'k','LineWidth',line_width)
xlabel('t/s', 'FontSize',ft_size);
ylabel('Receiving', 'FontSize',ft_size);
set(gca, 'FontSize', ft_size);
set(gca,'Xlim',xlim);
legend('Loop1','Loop2','Loop3','Loop4');