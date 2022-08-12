function [pWNorm,pWSphere] = testBearingVectorAndNormPlaneVector(K,P,yaw,pitch,roll,t)
%TESTBEARINGVECTORANDNORMPLANEVECTOR 此处显示有关此函数的摘要
%   使用归一化平面的点还是球面上的点进行三角化计算的深度含义不同，一个是z轴上的距离，另一个是路标点到相机中心距离；
% 但是最终计算出的路标点坐标都是正确的。
% 如果特征点距离相机很远，远大于两帧相对的水平运动，那么其深度误差将完全由像素噪声决定
R = angle2dcm(yaw,pitch,roll,'ZYX');
PInCurCam = R * P + t;
uvInRef = pinhole(K,P)
uvInCur = pinhole(K,PInCurCam) + [1;1] 

pInRefNorm = cam2Normlize(K,uvInRef);
pInCurNorm = cam2Normlize(K,uvInCur);
[flg,depth] = triangulatePoint(R,t,pInRefNorm,pInCurNorm);
if flg == 0 
   fprintf("Check R and t is not right for triangulation!\n");
   return;
end
pWNorm = pInRefNorm * abs(depth(1));
pInRefSphere = cam2Sphere(K,uvInRef);
pInCurSphere = cam2Sphere(K,uvInCur);
[flg2,depthSphere] = triangulatePoint(R,t,pInRefSphere,pInCurSphere);
if flg2 == 0 
   fprintf("Check R and t is not right for triangulation!\n");
   return;
end
pWSphere = pInRefSphere * abs(depthSphere(1));


end

