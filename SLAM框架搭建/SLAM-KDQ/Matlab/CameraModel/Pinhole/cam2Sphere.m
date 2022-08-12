function bearingVector = cam2Sphere(K,uv)
%CAM2 --- 将像素坐标转换到单位球面上
%   K -- 相机内参
%  uv -- 像素坐标
% bearingVector -- 球面坐标
pInNorm = cam2Normlize(K,uv);
bearingVector = pInNorm / norm(pInNorm);
end

