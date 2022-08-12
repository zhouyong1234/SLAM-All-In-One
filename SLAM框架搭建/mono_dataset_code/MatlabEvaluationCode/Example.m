
% load all sequences. 
% useful to evaluate many runs, as groundtruth file doesn't need to be
% loaded again every time.
allSeq = errorPerSequenceBenchmark('/home/engelj/bags/mocap_sequences');



% load only one sequence
singleSeq = struct;
singleSeq.name = 'sequence_01';
singleSeq.mocapRaw = importdata(['/home/engelj/bags/mocap_sequences/' singleSeq.name '/groundtruthSync.txt']);




%% 
sequence = allSeq{29};

% =============== Evaluate: ================
% INPUT:
% first argument: file name for estimated trajectory.
% second argument: sequence (struct with ".name" and ".mocapRaw" property, where ".mocapRaw" contains the data from "groundtruthSync.txt" )
% third argument: 0=don't plot. 1=plot (x,y,z), aligned. 2=plot top-down view
%
% OUTPUT:
% rmse = RMSE when just aligning the GT with the trajectory (e_\text{rmse})
% errTrafo = Sim(3) transformation transforming start- to end-segment (T_\text{drift})
% errAlign = Alignment Error (e_\text{align}).
% errR = rotation drift (e_r)
% errS = scale drift (e_s)
% absErrA = RMSE for start-segment only (as plotted in Fig. 12, left)
% absErrE = RMSE for end-segment only (as plotted in Fig. 12, right)
% nkf = number of poses in estimated trajectory.


[rmse, errTrafo, errAlign, errR, errS, abserrA, abserrE, nkf] = efficientEvalDrift(['examples/' sequence.name '.txt'], sequence, 2)

        
