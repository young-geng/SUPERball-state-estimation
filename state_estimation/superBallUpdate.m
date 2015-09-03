function superBallUpdate(superBall1,superBallUKFPlot1,tspan1,ax1,text1,barlength1,lines1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%create some persistent variables for objects and structs
persistent superBall superBallUKFPlot tspan allMeasureIndices ax i texth barlength lines
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
    lines = lines1;
    lastAngles = zeros(6,1);
else if nargin == 1
        i = i+1;
        msgData = superBall1;
        
        %%%%%%%%%%%%%Process message data %%%%%%%%%%%%%%%%%%%%%
        % Most of the processing is done before it reaches MATLAB
        rangingMeasures = msgData(1:end-6);
        isBar = [1, 22, 39, 52, 61, 66];
        isInternal = 1:(11+10+9+8+7+6+5+4+3+2+1);
        rangingMeasures(isInternal) = 0;
        rangingMeasures(isBar) = barlength + 3.9;
        
        
        %%%% SETS ALL MEASUREMENTS TO ZERO %%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        allAngleMeasures = msgData(end-5: end);
        isGoodAngle = ~isnan(allAngleMeasures);
        superBall.goodAngles = isGoodAngle;
        %disp(allAngleMeasures);
        angleMeasures = allAngleMeasures(isGoodAngle);
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
        nodes = superBallUKFPlot.nodePoints;
        updatePlot(superBallUKFPlot);
        for j = 1: 12
            texth(j).Position = nodes(j,:) + [0 0 0.1];
        end
        
        baseStationPoints = [
            -3.3800    0         0.3500;
            -2.3800   -0.9800    2.4300;
             0         0         0.3500;
            -0.7039   -2.1896    0.3500];
        lengthMeasures = [
    nodes(1,:);     baseStationPoints(1,:);    nodes(2,:);     baseStationPoints(1,:);
    nodes(3,:);     baseStationPoints(1,:);    nodes(4,:);     baseStationPoints(1,:);
    nodes(5,:);     baseStationPoints(1,:);    nodes(6,:);     baseStationPoints(1,:);
    nodes(7,:);     baseStationPoints(1,:);    nodes(8,:);     baseStationPoints(1,:);
    nodes(9,:);     baseStationPoints(1,:);    nodes(10,:);    baseStationPoints(1,:);
    nodes(11,:);    baseStationPoints(1,:);    nodes(12,:);    baseStationPoints(1,:);
    
    nodes(1,:);     baseStationPoints(2,:);    nodes(2,:);     baseStationPoints(2,:);
    nodes(3,:);     baseStationPoints(2,:);    nodes(4,:);     baseStationPoints(2,:);
    nodes(5,:);     baseStationPoints(2,:);    nodes(6,:);     baseStationPoints(2,:);
    nodes(7,:);     baseStationPoints(2,:);    nodes(8,:);     baseStationPoints(2,:);
    nodes(9,:);     baseStationPoints(2,:);    nodes(10,:);    baseStationPoints(2,:);
    nodes(11,:);    baseStationPoints(2,:);    nodes(12,:);    baseStationPoints(2,:);
    
    nodes(1,:);     baseStationPoints(3,:);    nodes(2,:);     baseStationPoints(3,:);
    nodes(3,:);     baseStationPoints(3,:);    nodes(4,:);     baseStationPoints(3,:);
    nodes(5,:);     baseStationPoints(3,:);    nodes(6,:);     baseStationPoints(3,:);
    nodes(7,:);     baseStationPoints(3,:);    nodes(8,:);     baseStationPoints(3,:);
    nodes(9,:);     baseStationPoints(3,:);    nodes(10,:);    baseStationPoints(3,:);
    nodes(11,:);    baseStationPoints(3,:);    nodes(12,:);    baseStationPoints(3,:);
    
    nodes(1,:);     baseStationPoints(4,:);    nodes(2,:);     baseStationPoints(4,:);
    nodes(3,:);     baseStationPoints(4,:);    nodes(4,:);     baseStationPoints(4,:);
    nodes(5,:);     baseStationPoints(4,:);    nodes(6,:);     baseStationPoints(4,:);
    nodes(7,:);     baseStationPoints(4,:);    nodes(8,:);     baseStationPoints(4,:);
    nodes(9,:);     baseStationPoints(4,:);    nodes(10,:);    baseStationPoints(4,:);
    nodes(11,:);    baseStationPoints(4,:);    nodes(12,:);    baseStationPoints(4,:);
    ];
        measureLines = [67:4:114, 68:4:114, 69:4:114, 70:4:114] ;

        for j = 1: 48
            texth(j+16).Position = (lengthMeasures(2*j-1,:) + lengthMeasures(2*j,:)*3)/4;
            if(isNewMeasurement(measureLines(j)))
            texth(j+16).String = num2str(rangingMeasures(measureLines(j))-3.9,2);
            else
                lengthMeasures(2*j-1,:) = lengthMeasures(2*j,:);
                texth(j+16).String = ' ';
            end
        end
        
        for j =1:4
        lines(j).XData = lengthMeasures(24*j+(-23:0),1); lines(j).YData = lengthMeasures(24*j+(-23:0),2); lines(j).ZData = lengthMeasures(24*j+(-23:0),3);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        x_Avg = mean( superBallUKFPlot.nodePoints(:,1));
        y_Avg = mean( superBallUKFPlot.nodePoints(:,2));
        lims = 2*barlength;
        xlim(ax(1),[-lims lims]+x_Avg)
        ylim(ax(1),[-lims lims]+y_Avg)
        
    else
        for j = 1
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

