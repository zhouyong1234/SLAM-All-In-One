function [mSeed,rSeed,lastMSeed] = testRovioFeature(rovioData,id,delays)
%TESTROVIOFEATURE 此处显示有关此函数的摘要
%   此处显示详细说明
mSeed = [];
rSeed = [];
lastMSeed = [];
dfDatas = extractRovioIdData(rovioData,id);
if isempty(dfDatas) 
    %fprintf("No this id data!\n");
    return;
end
len = length(dfDatas.id);
if len == 0 
    fprintf("No data input!\n");
    return;
end
seed = SeedInitializeWithRovioData(dfDatas,1);

for i = 1:len - 1
    meas.x = dfDatas.x(i);
    meas.tau2 = dfDatas.tau2(i);
    seed = DepthFilter(seed,meas,delays);
    pause(delays);
end

mSeed = seed;
rSeed = SeedInitializeWithRovioData(dfDatas,len);
fprintf("Id: %d ,Update count: %d\n",id,len - 1);
meas.x = dfDatas.x(len);
meas.tau2 = dfDatas.tau2(len);
lastMSeed = DepthFilter(seed,meas,1);
end

