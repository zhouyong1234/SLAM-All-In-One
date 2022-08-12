function seedNew = DepthFilter(seed,meas,delays)
%% DEPTHFILTER 深度滤波器 -- 特征点逆深度滤波采用Beta * Gaussian模型近似真实的含有外点的直方图分布
%   seed -  存储特征点深度的概率模型Beta*Gaussian
%        -  a : Beta内点数量
%        -  b : Beta外点数量
%        - depthRange : 逆深度范围(1/minDepth - 1/maxDepth ~= 1/minDepth)
%        -  mu: Gaussian均值
%        -  sigma2 : Gaussian方差
%   meas - 当前量测的高斯分布模型
%        -   x  : Gaussian均值
%        - tau2 : Gaussian方差
seedNew = [];
norm_scale = (seed.sigma2 + meas.tau2)^0.5;
if isnan(norm_scale)
    fprintf("norm is nan!");
    return 
end
%获得量测值在上一次深度高斯分布中的概率值
measPInLast = normpdf(meas.x,seed.mu,norm_scale);
% test %
measDepthErrInLast = inverseErr2DepthErr(seed.mu,norm_scale);
measDepthScale = inverseErr2DepthErr(meas.x,meas.tau2^0.5);
[maVecx,maVecy,masp] = plotNormalDistribution(1./meas.x,measDepthScale,1/meas.x,0);
[mVecx,mVecy,msp] = plotNormalDistribution(1./seed.mu,measDepthErrInLast,1/meas.x,0);
%当前量测的高斯分布和上一次高斯分布融合后的高斯噪声
s2 = 1/(1/seed.sigma2 + 1/meas.tau2);
%对应的期望
m = s2 * (seed.mu / seed.sigma2 + meas.x / meas.tau2);
%内点对应的概率
C1 = seed.a / (seed.a + seed.b) * measPInLast;
%外点对应的概率
C2 = seed.b / (seed.a + seed.b) * 1. / seed.depthRange;
%归一化概率
normC1C2 = C1 + C2;
C1 = C1 / normC1C2;
C2 = C2 / normC1C2;

f = C1 * (seed.a + 1) / (seed.a + seed.b + 1) + C2 * seed.a / (seed.a + seed.b + 1);
e = C1 * (seed.a + 1) * (seed.a + 2) / ((seed.a + seed.b + 1) * (seed.a + seed.b + 2)) ... 
  + C2 * seed.a * (seed.a + 1) / ((seed.a + seed.b + 1) * (seed.a + seed.b + 2));
% 更新种子
muNew = C1 * m + C2 * seed.mu;
seedNew.sigma2 = C1 * (s2 + m^2) + C2 * (seed.sigma2 + seed.mu^2) - muNew^2;
seedNew.mu = muNew;
seedNew.a = (e - f) / (f - e / f);
seedNew.b = seedNew.a * (1 - f) / f;
seedNew.d = 1 / seedNew.mu;
seedNew.depthRange = seed.depthRange;
if checkSeedNan(seedNew)
    fprintf("Cautious:Seed is nan!\n");
    return;
end
if delays == 0
    return;
end
%% plot
seedNewDepthErr = inverseErr2DepthErr(seedNew.mu,seedNew.sigma2^0.5);
[sVecx,sVecy,ssp] = plotNormalDistribution(seedNew.d,seedNewDepthErr,1./meas.x,0);
subplot(2,2,1);
plot(maVecx,maVecy,'r');
hold on;
plot(masp(1),masp(2),'Marker','*','MarkerSize',10,'Color','g');
hold on;
plot(mVecx,mVecy,'b');
hold on;
plot(msp(1),msp(2),'Marker','*','MarkerSize',10,'Color','g');
grid on;
legend('当前量测','当前期望','复合量测','复合映射期望');
title("当前测量映射到上一次深度的高斯分布图");
hold off;
subplot(2,2,2);
plot(sVecx,sVecy);
hold on;
plot(ssp(1),ssp(2),'Marker','*','MarkerSize',10,'Color','g');
grid on;
title("深度滤波器更新后的高斯分布");
hold off;
subplot(2,2,3);
plot(1,seedNew.a,'Marker','*','MarkerSize',10,'Color','g');
hold on;
plot(3,seedNew.b,'Marker','diamond','MarkerSize',10,'Color','r');
hold off;
subplot(2,2,4);
depthSkew = inverseErr2DepthErr(seedNew.mu,seedNew.sigma2^0.5)/seedNew.d;
plot(1,depthSkew,'Marker','*','MarkerSize',10,'Color','g');
hold off;
grid on;
title("深度不确定度");




end

