

Pw = [0;0;200];
R = angle2dcm(0,0,0);
K = [174,0,160;0,171,120;0,0,1];
uvRef = pinhole(K,Pw);
uvRefSphere = cam2Sphere(K,uvRef);
t = [10;0;0]; % [10;0;0];
Pc = R * Pw + t;
uvCur = pinhole(K,Pc);
uvCurSphere = cam2Sphere(K,uvCur)

depVec = [];
depEVec = [];
csVec = [];
iVec = [];
verBear = cross(uvRefSphere,uvCurSphere);
verBear = verBear / norm(verBear);
for i = 0 : pi/30 : pi/2
  Rn = angle2dcm(i,0,0,'ZYX');
  tNoise = Rn * t;
  cthe = verBear' * tNoise / norm(tNoise);
  theta = rad2deg(acos(cthe))
  [flgN,depthN,BN,detN,cs] = triangulatePoint(R,tNoise,uvRefSphere,uvCurSphere); 
  depEVec = [depEVec;(Pw(3) - depthN(1))/Pw(3)];
  iVec = [iVec;i * 57.29];
  depVec = [depVec;depthN(1)];
  csVec = [csVec;cs(3,3)];
end
close all;
subplot(2,1,1)
[hax,l1,l2] = plotyy(iVec,depVec,iVec,depEVec);
l1.Marker = '*';
l2.Marker = '*';
legend('解算深度','误差深度');
title('因为位置向量和bearvector不在一个平面导致的误差');
grid on;
subplot(2,1,2)
plot(iVec,csVec,'g*');
title('svd分解最小奇异值');
grid on;
fprintf("这个实验告诉我们：\n" + ...
        "1）如果位置向量不在bearvector组成的平面内，那么三者构成的矩阵行列式绝对不为0，且最小奇异值与位置向量相对平面的夹角成正比;\n" + ...
        "2) 这个夹角导致的深度误差与位置向量大小无关，只与这个夹角大小有关。\n");


