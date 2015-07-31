function superBallUpdate(superBall1,superBallDynamicsPlot1,superBallUKFPlot1,tspan1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%create some persistent variables for objects and structs
persistent superBall superBallDynamicsPlot superBallUKFPlot tspan


if nargin>1
    superBall = superBall1;
    superBallDynamicsPlot = superBallDynamicsPlot1;
    superBallUKFPlot = superBallUKFPlot1;
    tspan = tspan1;
end

%%%%%%%%%%%%%%%%%% update Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%% update superBall nodes %%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%Compute Command and update dynamics $%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dynamicsUpdate(superBall,tspan);
actualNodes =  superBall.ySim(1:end/2,:);
noise = randn(size(actualNodes))*0.05;
superBall.measurementUKFInput = actualNodes + noise;
ukfUpdate(superBall,tspan);
%disp( superBall.ySimUKF(1:end/2,:));

superBallDynamicsPlot.nodePoints = actualNodes + noise;
superBallUKFPlot.nodePoints = superBall.ySimUKF;
%disp( superBall.ySimUKF(1:end/2,:));
updatePlot(superBallDynamicsPlot);
updatePlot(superBallUKFPlot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drawnow %plot it up
end

