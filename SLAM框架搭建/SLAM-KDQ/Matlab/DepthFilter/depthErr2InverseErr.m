function inverseErr = depthErr2InverseErr(depth,depthErr)
%DEPTHERR2INVERSEERR 通过深度误差和深度大小来计算逆深度误差
%   depth     --  深度值
%   depthErr  --  深度误差
inverseErr = 0.5 * (1/max(1e-7,depth - depthErr) - 1.0/(depth + depthErr));
end

