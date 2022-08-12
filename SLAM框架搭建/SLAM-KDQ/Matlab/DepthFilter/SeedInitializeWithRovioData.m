function seed = SeedInitializeWithRovioData(rovioData,index)
%SEEDINITIALIZEWITHROVIODATA 此处显示有关此函数的摘要
%   此处显示详细说明
seed.a = rovioData.a(index);
seed.b = rovioData.b(index);
seed.mu = rovioData.mu(index);
seed.sigma2 = rovioData.sigma2(index);
seed.depthRange = rovioData.depthRange(index);
seed.d = 1 / seed.mu;
end

