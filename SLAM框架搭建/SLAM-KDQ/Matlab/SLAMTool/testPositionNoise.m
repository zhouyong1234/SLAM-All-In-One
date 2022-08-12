xNoise = 0.1;
yNoise = 0.1;
zNoise = 0.4;

pNoise = [xNoise,yNoise,zNoise; ...
          -xNoise,yNoise,zNoise;...
          xNoise,-yNoise,zNoise;...
          xNoise,yNoise,-zNoise;...
          -xNoise,-yNoise,zNoise;...
          -xNoise,yNoise,-zNoise;...
          xNoise,-yNoise,-zNoise;...
          -xNoise,-yNoise,-zNoise];


Pw = [0;0;30];
R = angle2dcm(0.1,0.2,0.3);
K = [174,0,160;0,171,120;0,0,1];

tMax = 30;
uvRef = pinhole(K,Pw);
uvRefSphere = cam2Sphere(K,uvRef);

depVec = [];
depNVec = [];
depEVec = [];
csVec = [];
iVec = [];
t = [1;0;0]; % [10;0;0];
Pc = R * Pw + t;
uvCur = pinhole(K,Pc);
uvCurSphere = cam2Sphere(K,uvCur);
[flg,depth,B,det,cs] = triangulatePoint(R,t,uvRefSphere,uvCurSphere); 
cs

for i = 1 : 1 : 8
  tNoise = t + pNoise(i)';
  [flgN,depthN,BN,detN,cs] = triangulatePoint(R,tNoise,uvRefSphere,uvCurSphere); 
  depN = BN * pNoise(i)';
  depNVec = [depNVec;depN(1)];
  depEVec = [depEVec;depthN(1) - depth(1)];
  iVec = [iVec;i];
  depVec = [depVec;depthN(1)];
   cs
end
plot(0,depth(1),'g*',iVec,depVec,'r*',iVec,depNVec,'b.',iVec,depEVec,'b+');
legend('真实深度','误差深度','Jacbin近似深度误差','真实深度误差');
grid on;

fprintf("这个实验说：1）当水平位置运动相对真实深度比较小的时候，位置误差对其影响很大；2）可以求位置扰动对深度的Jacbin与位置噪声相乘直接获得近似的深度误差。");






