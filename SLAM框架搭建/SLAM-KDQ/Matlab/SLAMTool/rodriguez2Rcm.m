function Rcm = rodriguez2Rcm(rotVec)
%RODRIGUEZ2RCM 此处显示有关此函数的摘要
%   此处显示详细说明

rn = norm(rotVec);
skewM = vec3d2SkewMat(rotVec);
if rn < 1e-4
   Rcm = eye(3) + skewM;
else
   Rcm = eye(3) + sin(rn) * skewM / rn + (1 - cos(rn)) * skewM * skewM / (rn^2);
end

end

