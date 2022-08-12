function pHomo = pointHomogeneous(p)
%POINTHOMOGENEOUS -- 获取点的齐次形式
%   此处显示详细说明
p_dims = size(p);
if p_dims(2) ~= 1
    fprintf("Please input vector or point instead of matrix!\n");
    return 
end
pHomo = [p;1];
end

