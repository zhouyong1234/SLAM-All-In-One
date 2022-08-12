function depthErr = inverseErr2DepthErr(inverseDepth,inverseDepthErr)
%INVERSEERR2DEPTHERR 根据逆深度和逆深度误差计算深度误差
%  事实上逆深度和
depthErr = inverseDepthErr / inverseDepth^2;
end

