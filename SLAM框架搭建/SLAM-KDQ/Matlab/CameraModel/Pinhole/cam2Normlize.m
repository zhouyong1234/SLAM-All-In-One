function pInNorm = cam2Normlize(K,uv)
%CAM2NORMLIZE --- 像素坐标转归一化平面坐标
% K -- 相机内参
% uv -- 像素坐标
% pInNorm -- 
k_dims = size(K);
uv_dims = size(uv);
if k_dims(1) ~= k_dims(2) || k_dims(1) ~= 3 || uv_dims(1) ~=2 || uv_dims(2) ~= 1
    fprintf("Please input right K and uv!\n")
    return;
end
uvHomo = [uv(1);uv(2);1];
pInNorm = inv(K) * uvHomo;
end

