function meas = MeasInitialize(depth,depthErr)
%% MEASINITIALIZE  深度测量值
%   此处显示详细说明
% depth -  量测值深度
% depthErr - 量测值标准差(深度误差不可以大于或者接近深度值一半，否则计算的逆深度不确定度和真实深度可能对不上)
meas.x = single(1/depth);
%对应逆深度标准差
tau = depthErr2InverseErr(depth,depthErr);
meas.tau2 = single(tau^2);
%这里测试逆深度模型标准差转到深度标准差是否和depthErr一致
%发现：不一致，当depthErr约大特别是大于一半的depth之后，二者就很不准，切depthErr大于depth之后逆深度标准差无法变
depErr = inverseErr2DepthErr(meas.x,tau);
%fprintf("请确保深度标准差小于深度一半，否则逆深度不确定度无法准确对应上深度不确定度.\n");
%fprintf("深度标准差：%f VS 逆深度转化到深度的标准差：%f\n",depthErr,depErr);
plotNormalDistribution(depth,depErr,depth,0);
end

