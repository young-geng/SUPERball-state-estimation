function superBallUpdate(superBall1,superBallCommandPlot1,superBallDynamicsPlot1,pStruct1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%create some persistent variables for objects and structs
persistent superBall superBallCommandPlot superBallDynamicsPlot pStruct


if nargin>1
    superBall = superBall1;
    superBallCommandPlot = superBallCommandPlot1;
    superBallDynamicsPlot = superBallDynamicsPlot1;
    pStruct = pStruct1;
end

%%%%%%%%%%%%%%%%%% update Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set variables from persistent structure 
tspan = pStruct.tspan;
% set variables from sliders below


%%%%%%%%%%%%%% update spine nodes in command plot and spine object %%%%%%%%%%%%%%%%%%%%%
%superBall.nodePoints = spineNodes;
%superBallCommandPlot.nodePoints = spineNodes;
updatePlot(superBallCommandPlot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%Compute Command and update dynamics if feasible command is generated%%%%%%%
ukfUpdate(superBall,tspan);
%disp( superBall.ySimUKF(1:end/2,:));
superBallDynamicsPlot.nodePoints = superBall.ySimUKF(1:end/2,1:3);
%disp( superBall.ySimUKF(1:end/2,:));
updatePlot(superBallDynamicsPlot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drawnow %plot it up
end

