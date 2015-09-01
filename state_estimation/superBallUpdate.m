function superBallUpdate(superBall1,superBallUKFPlot1,tspan1,ax1,text1,barlength1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%create some persistent variables for objects and structs
persistent superBall superBallUKFPlot tspan allMeasureIndices ax i texth barlength

if nargin>1
    i = 0;
    superBall = superBall1;
    superBallUKFPlot = superBallUKFPlot1;
    tspan = tspan1;
    internalMeasureIndices = [1*ones(1,11), 2*ones(1,10), 3*ones(1,9), 4*ones(1,8), 5*ones(1,7), 6*ones(1,6), 7*ones(1,5), 8*ones(1,4), 9*ones(1,3), 10*ones(1,2), 11*ones(1,1);
                              2:12,         3:12,         4:12,        5:12,        6:12,        7:12,        8:12,        9:12,        10:12,       11:12,        12:12];
    externalMeasureIndices = [1*ones(1,4), 2*ones(1,4), 3*ones(1,4), 4*ones(1,4), 5*ones(1,4), 6*ones(1,4), 7*ones(1,4), 8*ones(1,4), 9*ones(1,4), 10*ones(1,4), 11*ones(1,4), 12*ones(1,4);
                              13:16,       13:16,       13:16,       13:16,       13:16,       13:16,       13:16,       13:16,       13:16,       13:16,        13:16,        13:16];    
    allMeasureIndices = [internalMeasureIndices externalMeasureIndices];
    ax = ax1;
    texth = text1;
    barlength = barlength1;
else if nargin == 1
        i = i+1;
        msgData = superBall1;

        %%%%%%%%%%%%%Process message data %%%%%%%%%%%%%%%%%%%%%
        % Most of the processing is done before it reaches MATLAB
        rangingMeasures = msgData(1:end-6);
         fprintf('%7.2f', rangingMeasures - 3.9)
            %disp(round((z1-z)',2))
            fprintf('\r\n')
          %11+10%  %9+8%  %7+6%  %5+4%  %3+2%
        isBar = [1, 22, 39, 52, 61, 66];
        %isInternal = 1:(11+10+9+8+7+6+5+4+3+2+1);
        %rangingMeasures(:) = 0 ;%(isInternal) = 0;
        rangingMeasures(isBar) = barlength + 3.9; 
        
        %%%% SETS ALL MEASUREMENTS TO ZERO %%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        allAngleMeasures = msgData(end-5: end);
        allAngleMeasures(isnan(allAngleMeasures)) = 0;
        angleMeasures = allAngleMeasures;
        rangingMeasures(isnan(rangingMeasures)) = 0;
        isNewMeasurement = rangingMeasures>0 & rangingMeasures<12 ;
        
        %%%%%%%%%%%%Input Measurements and commands %%%%%%%%%%%%
        %superBall.simStructUKF.stringRestLengths; %TODO: Need to implement this
        
        superBall.lengthMeasureIndices = allMeasureIndices(:,isNewMeasurement);
        goodLengths = rangingMeasures(isNewMeasurement) - 3.9;
        superBall.measurementUKFInput = [angleMeasures; goodLengths]; %UKF measures
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        ukfUpdate(superBall,tspan);
        superBallUKFPlot.nodePoints = superBall.ySimUKF(1:end/2,:);
        updatePlot(superBallUKFPlot);
        for i = 1: 12
            texth(i).Position = superBall.ySimUKF(i,:) + [0 0 0.1];
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        x_Avg = mean( superBallUKFPlot.nodePoints(:,1));
        y_Avg = mean( superBallUKFPlot.nodePoints(:,2));
        lims = 2*barlength;
        xlim(ax(1),[-lims lims]+x_Avg)
        ylim(ax(1),[-lims lims]+y_Avg)
      
    else
        for i = 1
            disp('huh?')
%         i = i+1;
%         %%%%%%%%%%%%%%%%%% update Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
%         %%%%%%%%%%%%%% update superBall nodes %%%%%%%%%%%%%%%%%%%%%
% 
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%         %%%%%%%%%%%%Compute Command and update dynamics $%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         if(mod(i,1)==0)
%         currentWorkingLengths = unique(round(rand(00,1)*107))+1;
%         else
%             currentWorkingLengths = unique(round(rand(0,1)*107))+1;
%         end
%         %disp(length(currentWorkingLengths))
%         workingMeasureIndices = allMeasureIndices(:,currentWorkingLengths);
%         superBall.lengthMeasureIndices = [bars workingMeasureIndices];
%         dynamicsUpdate(superBall,tspan);
%         actualNodes =  superBall.ySim(1:end/2,:);
%         barVec = actualNodes(bars(1,:),:) - actualNodes(bars(2,:),:);
%         barNorm = sqrt(barVec(:,1).^2 + barVec(:,2).^2 + barVec(:,3).^2);
%         barAngleFromVert = acos(barVec(:,3:3:end)./barNorm);
% 
%         LI =  workingMeasureIndices;
%         lengthNoise = randn(size(LI,2),1)*0.05;
%         tiltNoise = 5/180*pi*randn(6,1);
%         yyPlusBase = [actualNodes; superBall.baseStationPoints];
%         allVectors = (yyPlusBase(LI(1,:),:) - yyPlusBase(LI(2,:),:)).^2;
%         z = sqrt(sum(allVectors,2));
%         superBall.measurementUKFInput =[barAngleFromVert+tiltNoise; 1.7*ones(6,1); (z + lengthNoise + lambdaErrors(currentWorkingLengths))];
%         ukfUpdate(superBall,tspan);
% 
%         superBallDynamicsPlot.nodePoints = actualNodes ;
%         superBallUKFPlot.nodePoints = superBall.ySimUKF(1:end/2,:);
%         updatePlot(superBallDynamicsPlot);
%         updatePlot(superBallUKFPlot);
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         lims = 1.2*1.7;
%         x_Avg = mean(actualNodes(:,1));
%         y_Avg = mean(actualNodes(:,2));
%         xlim(ax(1),[-lims lims]+x_Avg)
%         ylim(ax(1),[-lims lims]+y_Avg)
%         xlim(ax(2),[-lims lims]+x_Avg)
%         ylim(ax(2),[-lims lims]+y_Avg)
%         drawnow  %plot it up
        end
    end
end
end

