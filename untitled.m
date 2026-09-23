% prr smoothing
% 
load('link_quality_80.mat');
link3f;
rate = zeros(5000,1);
rate(1) = 1;
window_size = 200;

for i = 2:1:5000
    if i<=window_size
        rate(i) = mean(link3f(1:i));
    else
        rate(i) = mean(link3f(i-window_size:i));
    end
end
figure;
plot(1:1:5000,rate);