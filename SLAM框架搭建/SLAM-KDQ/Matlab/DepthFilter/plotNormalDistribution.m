function [xVec,yVec,sp] = plotNormDistribution(mu,sigma,s,flg)
%% PLOTNORMDIST 画高斯分布概率密度图
%      mu -- 高斯分布均值
%   sigma -- 高斯分布标准差
%       s -- x轴上感兴趣的点
xVec =[];
yVec = [];
sp = [];
cell = 10 * sigma / 100;
for x = mu - 5*sigma : cell : mu + 5 * sigma
    y = normpdf(x,mu,sigma);
    xVec = [xVec;x];
    yVec = [yVec;y];
end
sp = [sp;s];
sp = [sp;normpdf(s,mu,sigma)];
if flg == 0 
    return 
end
plot(xVec,yVec,'r');
hold on
plot(s,sp,'Marker','*','MarkerSize',10,'Color','g');
hold off
grid on;
legend('pdf');
title('normal distribution');
end

