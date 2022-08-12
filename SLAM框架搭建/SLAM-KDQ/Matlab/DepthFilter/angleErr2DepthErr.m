function depthErr = angleErr2DepthErr(trc,bearVec,z,angleErr)
%UNTITLED 根据角度bearingVector的角度误差计算深度误差
%   此处提供详细说明
a = z * bearVec - trc;
tNorm = norm(trc);
aNorm = norm(a);
alpha = acos(bearVec' * trc / tNorm)
beta = acos(a' * (-trc)/(tNorm * aNorm))
betaPlus = beta + angleErr;
gammaPlus = pi - alpha - betaPlus
zPlus = tNorm * sin(betaPlus) / sin(gammaPlus);
depthErr = zPlus - z;
end