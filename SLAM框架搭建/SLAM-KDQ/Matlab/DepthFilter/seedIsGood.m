function goodFlg = seedIsGood(seed,inlierThr)
%SEEDISGOOD 此处显示有关此函数的摘要
%   此处显示详细说明
inlierP = seed.a / (seed.a + seed.b);
if inlierP > inlierThr && seed.sigma2^0.5 < seed.depthRange/200
    goodFlg = 1;
else 
    goodFlg = 0;
end
end

