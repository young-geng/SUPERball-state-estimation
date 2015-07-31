function superBallUpdate(superBall1,superBallCommandPlot1,superBallDynamicsPlot1,tspan1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%create some persistent variables for objects and structs
persistent superBall superBallCommandPlot superBallDynamicsPlot tspan


if nargin>1
    superBall = superBall1;
    superBallCommandPlot = superBallCommandPlot1;
    superBallDynamicsPlot = superBallDynamicsPlot1;
    tspan = tspan1;
end

%%%%%%%%%%%%%%%%%% update Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%% update superBall nodes in command plot and superBall object %%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%Compute Command and update dynamics if feasible command is generated%%%%%%%
ukfUpdate(superBall,tspan);
%disp( superBall.ySimUKF(1:end/2,:));
yy = superBall.ySimUKF;
yy = [yy(1:end/2,1:3); yy(1:end/2,(1:3)+3*35); yy(1:end/2,(1:3)+3*70); yy(1:end/2,(1:3)+3*105); yy(1:end/2,(1:3)+3*143)];
superBallDynamicsPlot.nodePoints = yy;
%disp( superBall.ySimUKF(1:end/2,:));
updatePlot(superBallDynamicsPlot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drawnow %plot it up
end

