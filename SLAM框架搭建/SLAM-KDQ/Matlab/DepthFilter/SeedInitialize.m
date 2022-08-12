function seed = SeedInitialize(depth,minDepth,a,b)
%% SEEDINITIALIZE 初始化特征点逆深度种子的Beta*Gaussian模型
%      depth - 特征点初始深度
%   minDepth - 特征点可能的最小深度
%          a - Beta分布内点初始数量（a/(a+b)表示内点概率）
%          b - Beta分布外点初始数量
seed.a = single(a);
seed.b = single(b);
seed.d = single(depth);
seed.mu = single(1./depth);
seed.depthRange = single(1/minDepth);
% sigma2逆深度方差
seed.sigma2 = single(seed.depthRange^2 / 36); 
end

