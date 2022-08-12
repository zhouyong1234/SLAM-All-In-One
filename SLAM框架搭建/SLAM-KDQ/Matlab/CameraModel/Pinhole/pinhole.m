function uv = pinhole(K,P)
%PINHOLE 此处显示有关此函数的摘要
%   此处显示详细说明
% K -- 相机内参[fx,0,u0;0,fy,v0;0,0,1];
% P -- 相机坐标系下点坐标
k_dims = size(K);
p_dims = size(P);
if k_dims(1) ~= k_dims(2) || k_dims(1) ~= 3 || p_dims(1) ~=3 || p_dims(2) ~= 1
    fprintf("Please input right K and P!\n")
    return;
end
pNormInZ = P / P(3);
pixel = K * pNormInZ;
uv = [pixel(1);pixel(2)];
end

