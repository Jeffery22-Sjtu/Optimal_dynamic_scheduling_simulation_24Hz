%% Draw superframe
xlim=[2,3];
slot_length = (1/16)/7;
for tt = floor(xlim(1)*16)/16:(1/16):floor(xlim(2)*16)/16
    for i=1:6
    rectangle('Position',[tt+i*slot_length 0 slot_length 1])
    hold on;
    end
    for i=0:0
    beacon=rectangle('Position',[tt+i*slot_length 0 slot_length 1],'FaceColor',[0 .5 .5],'LineStyle','none');
    hold on;
    end
end