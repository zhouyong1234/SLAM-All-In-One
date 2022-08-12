%即便
Pw = [0;0;30];
K = [174,0,160;0,171,120;0,0,1];
uvRef = pinhole(K,Pw);
uvRefSphere = cam2Sphere(K,uvRef);
detVec = [];
depVec = [];
tVec = [];
for i = 1 : 1 :30
  t = [10;0;0]; 
  R = angle2dcm(0.5 * rand,0.5 * rand, 0.5 *rand);
  Pc = R * Pw + t;
  uvCur = pinhole(K,Pc);
  uvCurSphere = cam2Sphere(K,uvCur);
  [flg,depth,B,det] = triangulatePoint(R,t,uvRefSphere,uvCurSphere); 
  detVec = [detVec;det];
  tVec = [tVec;i];
  if flg == 0
      depth = -1;
  end
  depVec = [depVec;depth(1)];
end
subplot(2,1,1);
plot(tVec,detVec,'r');
subplot(2,1,2);
plot(tVec,depVec,'g');




