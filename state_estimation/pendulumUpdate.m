function pendulumUpdate(pendulum1,pendulumDynamicsPlot1,pendulumUKFPlot1,tspan1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%create some persistent variables for objects and structs
persistent pendulum pendulumDynamicsPlot pendulumUKFPlot tspan allMeasureIndices


if nargin>1
    pendulum = pendulum1;
    pendulumDynamicsPlot = pendulumDynamicsPlot1;
    pendulumUKFPlot = pendulumUKFPlot1;
    tspan = tspan1;
    allMeasureIndices = [2*ones(1,5);
                         3:7];
end

%%%%%%%%%%%%%%%%%% update Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%% update pendulum nodes %%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%Compute Command and update dynamics $%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%currentWorkingLengths = unique(round(rand(72,1)*107))+1;
workingMeasureIndices = allMeasureIndices;
pendulum.lengthMeasureIndices = workingMeasureIndices;
dynamicsUpdate(pendulum,tspan);
actualNodes =  pendulum.ySim(1:end/2,:);
LI = workingMeasureIndices;
noise = randn(size(LI,2),1)*0.05;
yyPlusBase = [actualNodes; pendulum.baseStationPoints];

allVectors = (yyPlusBase(LI(1,:),:) - yyPlusBase(LI(2,:),:)).^2;
z = sqrt(sum(allVectors,2));
pendulum.measurementUKFInput = z + noise;
ukfUpdate(pendulum,tspan);

pendulumDynamicsPlot.nodePoints = actualNodes ;
pendulumUKFPlot.nodePoints = pendulum.ySimUKF;
updatePlot(pendulumDynamicsPlot);
updatePlot(pendulumUKFPlot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drawnow %plot it up
end

