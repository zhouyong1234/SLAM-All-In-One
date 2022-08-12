function [rmse, errTrafo, errAlign, errR, errS, abserrA, abserrE, nkf] = efficientEvalDrift( benchmark, sequence, plotfig )

errAlign=inf;
errR=inf;
errS=inf;
abserrA=inf;
abserrE=inf;
rmse=inf;
errTrafo=inf(4);
nkf=0;



mocapRaw = sequence.mocapRaw;
estimatedTrajectory = importdata([benchmark]);


if(size(estimatedTrajectory,1)==0)
    ['NO DATA ' sequence.name]
    return
end


% sort, just in case they are not sorted.
[A B] = sort(estimatedTrajectory(:,1));
estimatedTrajectory = estimatedTrajectory(B,:);

% default: don't plot
if(nargin < 3 )
    plotfig=0;
end


% if there is any NAN, something went wrong.
if sum(sum(isnan(estimatedTrajectory))) > 0
    ['IS NAN' sequence.name]
    return
end

estimatedPosition = estimatedTrajectory(:,2:4);
estimatedTimes = estimatedTrajectory(:,1);
nkf = size(estimatedTimes,1);

% assiciate via time-stamps. GT needs to have same timestamps (+- 0.001s) as estimated poses.
gtPos = zeros(size(estimatedTimes,1),3);
gtID = 1;
for i=1:size(estimatedTimes,1)
    while(estimatedTimes(i) - mocapRaw(gtID,1) > 0.001 && gtID < size(mocapRaw,1))
        gtID = gtID+1;
    end
    
    if(abs(estimatedTimes(i) - mocapRaw(gtID,1)) > 0.001)
        'ERROR, cannot associate frame well'
	return
    end
    gtPos(i,1:3) = mocapRaw(gtID,2:4);
end

% eval and align segments are assumed to be half / half.
nframes = size(mocapRaw,1);
timesAlign =  [mocapRaw(1,1) mocapRaw(floor(nframes/2),1)];
timesEval =  [mocapRaw(ceil(nframes/2),1) mocapRaw(nframes,1)];


% align start segment
lsdFramesAlign = (estimatedTimes >= timesAlign(1)) & (estimatedTimes <= timesAlign(2)) & (~isnan(gtPos(:,1)));
estimatedPositionAlign = estimatedPosition(lsdFramesAlign,:);
gtPosAlign = gtPos(lsdFramesAlign,:);


lsdFramesEval = (estimatedTimes >= timesEval(1)) & (estimatedTimes <= timesEval(2)) & (~isnan(gtPos(:,1)));
estimatedPositionEval = estimatedPosition(lsdFramesEval,:);
gtPosEval = gtPos(lsdFramesEval,:);


% align (7DoF)
if(size(lsdFramesEval,1)==0 || size(lsdFramesAlign,1)==0)
    ['IS INCOMPLETE' sequence.name]
    return
end



[ abserrE, RE, tE, scaleE ] = AlignSimEfficient( gtPosEval, estimatedPositionEval );
[ abserrA, RA, tA, scaleA ] = AlignSimEfficient( gtPosAlign, estimatedPositionAlign );




if(isnan(abserrE) || isnan(abserrA) || isnan(scaleE) || isnan(scaleA))
    ['IS NAN' sequence.name]
    return
end





% get sequence aligned by EVAL.
estimatedPositionE_aligned = scaleE * estimatedPosition * RE' + repmat(tE', size(estimatedPosition,1), 1);

% get sequence aligned by ALIGN.
estimatedPositionA_aligned = estimatedPosition * scaleA * RA' + repmat(tA', size(estimatedPosition,1), 1);

errS = scaleA/scaleE;  % as factor
errorquat = dcm2quat( RE*RA');
errR = 2*acos(errorquat(1))*180/pi;      % as degree
errAlign = (sum(sum((estimatedPositionE_aligned-estimatedPositionA_aligned).^2)) / size(estimatedPosition,1))^0.5;

if( sum(sum(isnan(RE+RA))) > 0)
    errTrafo=inf(4);
else
    errTrafo = [scaleE*RE tE; 0 0 0 1] * [scaleA*RA tA; 0 0 0 1]^-1;      % as degree
end


[ rmse, ~, ~, ~ ] = AlignSimEfficient( [gtPosAlign; gtPosEval], [estimatedPositionAlign; estimatedPositionEval] );






if(plotfig==1)
    clf
    hold on
    plot(estimatedTimes-estimatedTimes(1), estimatedPositionA_aligned,'blue','LineWidth',2);
    plot(estimatedTimes-estimatedTimes(1), estimatedPositionE_aligned,'red','LineWidth',2);
    plot(estimatedTimes-estimatedTimes(1), gtPos,'green','LineWidth',3,'LineStyle','--');
    grid on
    axis([0 estimatedTimes(end)-estimatedTimes(1) min(min(min(estimatedPositionE_aligned)),min(min(estimatedPositionA_aligned)))-4 max(max(max(estimatedPositionE_aligned)),max(max(estimatedPositionA_aligned)))+4])
end


if(plotfig==2)

H = ([estimatedPositionE_aligned; estimatedPositionA_aligned])' * ([estimatedPositionE_aligned; estimatedPositionA_aligned]);
[U,S,V] = svd(H);
R = V*U';
if det(R) < 0
    V(:,3) = V(:,3) * -1;
    R = V*U';
end


    estimatedPositionE_aligned_rot = estimatedPositionE_aligned*U;
    estimatedPositionA_aligned_rot = estimatedPositionA_aligned*U;
    gtPos_rot = gtPos*U;

    
    d1=2;
    d2=1;
    
    n=size(gtPos_rot,1);
    clf
    hold on
    plot(gtPos_rot(1:(n/2),d1), gtPos_rot(1:(n/2),d2),'green','LineWidth',3);
    plot(gtPos_rot((n/2):end,d1), gtPos_rot((n/2):end,d2),'green','LineWidth',3);
    plot(estimatedPositionE_aligned_rot(:,d1), estimatedPositionE_aligned_rot(:,d2),'red','LineWidth',2)
    plot(estimatedPositionA_aligned_rot(:,d1), estimatedPositionA_aligned_rot(:,d2),'blue','LineWidth',2)
    axis equal
    grid on
end

end

 