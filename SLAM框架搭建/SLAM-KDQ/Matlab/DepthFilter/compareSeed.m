function sameFlg = compareSeed(s1,s2)
%COMPARESEED 此处显示有关此函数的摘要
if abs(s1.a - s2.a) > 2  || abs(s1.b - s2.b) > 2 || abs(s1.d - s2.d) > 0.1 * abs(s2.d) || abs(s1.sigma2 - s2.sigma2) > 1e-4
    sameFlg = 0;
else
    sameFlg = 1;
end
end

