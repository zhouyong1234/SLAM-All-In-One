
Pw = [0;0;100];
R = angle2dcm(0.1,0.2,0.3);
K = [174,0,160;0,171,120;0,0,1];

tMax = 30;
uvRef = pinhole(K,Pw);
uvRefSphere = cam2Sphere(K,uvRef);
detVec = [];
depVec = [];
tVec = [];
for i = 0.01 : 0.02 : 1
  t = [i;0;0]; 
  Pc = R * Pw + t;
  uvCur = pinhole(K,Pc);
  uvCurSphere = cam2Sphere(K,uvCur);
  [flg,depth,B,det] = triangulatePoint(R,t,uvRefSphere,uvCurSphere);
  if flg == 0
    depth = -1;
  end
  B
  tVec = [tVec;i];
  detVec = [detVec;det];
  depVec = [depVec;depth(1)];
end
subplot(2,1,1);
plot(tVec,detVec,'r');
subplot(2,1,2);
plot(tVec,depVec,'g');
fprintf("这个实验说明：1）位置移动越大，则求解的行列式值越大（该值与旋转矩阵无关），深度值约可靠，可以用该值来大体确定深度置信度；2)位置移动越大，B矩阵也约小，B矩阵对应位置误差相对深度的Jacbin，则说明位置约大，位置误差对深度影响越小。")


