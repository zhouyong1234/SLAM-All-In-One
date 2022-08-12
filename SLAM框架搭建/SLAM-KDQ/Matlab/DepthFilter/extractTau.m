function idTau = extractTau(tau,id,plotEnable)
%EXTRACTTAU 此处显示有关此函数的摘要
%   此处显示详细说明

len = length(tau.id);
idTau = [];
sizeVec = [];
j = 1;
lostCount = 0;

for i = 1 : len
   if tau.id(i) == id 
    idTau.id(j) = tau.id(i);
    idTau.z(j) = tau.z(i);
    idTau.angle(j) = tau.angle(i);
    idTau.tau(j) = tau.tau(i);
    idTau.angleNoise(j) = tau.angleNoise(i);
    sizeVec = [sizeVec;j];
    j = j + 1;
    lostCount = lostCount - 1;
   end
   if j > 1 
    lostCount = lostCount + 1;
   end
   if lostCount > 5000
       break;
   end
end
if isempty(idTau)
fprintf("Id = %d,no this data!\n",id);
return;
end
if nargin < 3 || plotEnable ~= 1
    return 
end
 
close all;
subplot(2,1,1);
plot(sizeVec,idTau.z,'g*',sizeVec,idTau.tau,'r*');
legend('depth','tau');
grid on;
subplot(sizeVec,idTau.angle,'g*',sizeVec,idTau.angleNoise,'r*');
legend('angle','angleNoise');
end


