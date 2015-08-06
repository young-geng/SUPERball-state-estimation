function superBallUpdate(superBall1,superBallDynamicsPlot1,superBallUKFPlot1,tspan1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%create some persistent variables for objects and structs
persistent superBall superBallDynamicsPlot superBallUKFPlot tspan allMeasureIndices


if nargin>1
    superBall = superBall1;
    superBallDynamicsPlot = superBallDynamicsPlot1;
    superBallUKFPlot = superBallUKFPlot1;
    tspan = tspan1;
    allMeasureIndices = [2*ones(1,1), 3*ones(1,2), 4*ones(1,3), 5*ones(1,4), 6*ones(1,5), ...
                7*ones(1,6), 8*ones(1,7), 9*ones(1,8), 10*ones(1,9),11*ones(1,10),12*ones(1,11)...
                13*ones(1,12), 14*ones(1,12), 15*ones(1,12), 16*ones(1,12);
                1, 1:2, 1:3, 1:4, 1:5, 1:6, 1:7, 1:8,  1:9,1:10,1:11, 1:12, 1:12, 1:12, 1:12];
    allMeasureIndices(:,[1 6 15 28 45 66]) = []; %eliminate bar measures
end

%%%%%%%%%%%%%%%%%% update Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%% update superBall nodes %%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%Compute Command and update dynamics $%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

currentWorkingLengths = unique(round(rand(72,1)*107))+1;
workingMeasureIndices = allMeasureIndices(:,currentWorkingLengths);
superBall.lengthMeasureIndices = workingMeasureIndices;
dynamicsUpdate(superBall,tspan);
actualNodes =  superBall.ySim(1:end/2,:);
LI = workingMeasureIndices;
noise = randn(size(LI,2),1)*0.05;
yyPlusBase = [actualNodes; superBall.baseStationPoints];
            allVectors = (yyPlusBase(LI(1,:),:) - yyPlusBase(LI(2,:),:)).^2;
z = sqrt(sum(allVectors,2));
disp(z)
superBall.measurementUKFInput = z + noise;
ukfUpdate(superBall,tspan);

superBallDynamicsPlot.nodePoints = actualNodes ;
superBallUKFPlot.nodePoints = superBall.ySimUKF;
updatePlot(superBallDynamicsPlot);
updatePlot(superBallUKFPlot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drawnow %plot it up
end

