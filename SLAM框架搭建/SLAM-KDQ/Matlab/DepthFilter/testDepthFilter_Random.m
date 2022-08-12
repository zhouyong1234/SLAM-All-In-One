seed = SeedInitialize(30,10,10,20);
%% 如果一开始不能确定数据的可靠性，可以将beta分布的外点概率提高，比如这里b=10，不如b=20最终的精度高
% 前面10个数据是噪音较大的错误量测
d = [100,10;120,20;98,30;86,25;25,10;30,15;98,43;120,30;87,17;132,50];
for i = 1 : 1: length(d)
meas = MeasInitialize(d(i,1),d(i,2));
seed = DepthFilter(seed,meas);
pause(1)
end
pause(3)
for i = 1: 1: 10
   dep = 95 + unidrnd(10);
   dErr = 20;
   meas = MeasInitialize(dep,dErr);
   seed = DepthFilter(seed,meas);
   pause(1)
end

seed