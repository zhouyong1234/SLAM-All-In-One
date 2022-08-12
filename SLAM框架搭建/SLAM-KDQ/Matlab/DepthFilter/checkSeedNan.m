function nanflg = checkSeedNan(seed)
%CHECKSEEDNAN 此处显示有关此函数的摘要
%   此处显示详细说明
if isnan(seed.a) || isnan(seed.b) || isnan(seed.mu) || isnan(seed.sigma2) 
    nanflg = true;
else
    nanflg = false;
end

end

