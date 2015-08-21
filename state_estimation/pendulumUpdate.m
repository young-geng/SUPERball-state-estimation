function pendulumUpdate(vec,pendulum1,pendulumDynamicsPlot1,pendulumUKFPlot1,tspan1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%create some persistent variables for objects and structs
persistent pendulum pendulumDynamicsPlot pendulumUKFPlot tspan allMeasureIndices


if nargin>1
    pendulum = pendulum1;
    pendulumDynamicsPlot = pendulumDynamicsPlot1;
    pendulumUKFPlot = pendulumUKFPlot1;
    tspan = tspan1;
    allMeasureIndices = [1 2*ones(1,5);
                         2 3:7];
end

disp(vec')
indies = (2:1:6)';
currentWorkingLengths = [1; indies(vec(:,2)>0)]; % unique(round(rand(5,1)*4))+1;
%disp(currentWorkingLengths)
workingMeasureIndices = allMeasureIndices(:,currentWorkingLengths);
pendulum.lengthMeasureIndices = workingMeasureIndices;
%dynamicsUpdate(pendulum,tspan);
%actualNodes =  pendulum.ySim(1:end/2,:);
LI = workingMeasureIndices;
noise = randn(size(LI,2),1)*0.05;
%yyPlusBase = [actualNodes; pendulum.baseStationPoints];

%allVectors = (yyPlusBase(LI(1,:),:) - yyPlusBase(LI(2,:),:)).^2;
%zz = sqrt(sum(allVectors,2));
%disp(zz)
z = [2.4-0.73; vec((vec(:,2)>0),1)];
pendulum.measurementUKFInput = z + noise;
ukfUpdate(pendulum,tspan);

%pendulumDynamicsPlot.nodePoints = actualNodes ;
pendulumUKFPlot.nodePoints = [pendulum.ySimUKF(1:2,:); pendulum.baseStationPoints];
%updatePlot(pendulumDynamicsPlot);
updatePlot(pendulumUKFPlot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drawnow %plot it up
end

