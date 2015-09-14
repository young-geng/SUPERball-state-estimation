
clc
clear all
close all

barLength = 1.7;
totalSUPERballMass = 21;    % kg
barSpacing = barLength/4;
lims = 2*barLength;
gravity = 9.81;             % m/s^2
tspan =1/10;                % time between plot updates in seconds
delT = 0.001;               % timestep for dynamic sim in seconds
delTUKF  = 0.005;
Kp = 998;                   %passive string stiffness in Newtons/meter
Ka = 3150;                  %active string stiffness in Newtons/meter
preTension = 100;                   % how much force to apply to each cable in Newtons
nodalMass = (totalSUPERballMass/12)*ones(12,1);
Cp = 30;                    % damping constant, too lazy to figure out units.
Ca = 50;                    % constant for passive and active springs
barDamping = Cp/100*ones(6,1);
F = zeros(12,3);
stringStiffness = [Ka*ones(12,1); Kp*ones(12,1)];   % First set of 12 are acuated springs, second are passive
barStiffness = 100000*ones(6,1);
stringDamping = [Ca*ones(12,1); Cp*ones(12,1)];     % string damping vector
global state;
global updateVel_all;
global goodRestlengths_all;
global hvid;
global f;

movieFrames = VideoWriter('test.avi');
options = optimoptions('quadprog','Algorithm',  'interior-point-convex','Display','off');

baseStationPoints = [
     5.3500    1.2500    0.3500;
     2.4200    1.0000    2.6600;
     0              0    0.3500;
    -0.9600    2.3500    0.3500];
labels = {'1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16'};


nodes = [
    barSpacing      0               -barLength*0.5;
    barSpacing      0                barLength*0.5;
    0               barLength*0.5    barSpacing;   %5
    0              -barLength*0.5    barSpacing;
    barLength*0.5   barSpacing       0;
   -barLength*0.5   barSpacing       0;
   -barSpacing      0                barLength*0.5;
   -barSpacing      0               -barLength*0.5;             %8      
    0               barLength*0.5   -barSpacing;
    0              -barLength*0.5   -barSpacing;
    barLength*0.5  -barSpacing       0;
   -barLength*0.5  -barSpacing       0;
    ];

HH  = makehgtform('axisrotate',[0 1 0],-0.6);
HH  = makehgtform('axisrotate',[1 0 0],-0.6)*HH;
nodes = (HH(1:3,1:3)*nodes')';
nodes(:,3) = nodes(:,3) - min(nodes(:,3));
nodes(:,2) = nodes(:,2) + 0.95 ;
nodes(:,1) = nodes(:,1) + 0.95 ;

bars = [1:2:11;
    2:2:12];
strings = [1  2 3 4 5 6 7 8  9 11 12  10 1 1 11 11 10 10 3 3 7  7 6 6;
           11 5 7 2 9 3 6 12 8 10 4   1 9 5  2  4  12  8 5 2 4 12 8 9];

stringRestLength = [(1-(preTension/Ka))*ones(12,1)*norm(nodes(2,:)-nodes(5,:)); %active
                                                              (1-(preTension/Kp))*ones(12,1)*norm(nodes(1,:)-nodes(9,:))]; %passive

lengthMeasureIndices = [
    2*ones(1,1), 3*ones(1,2), 4*ones(1,3), 5*ones(1,4), 6*ones(1,5), ...
    7*ones(1,6), 8*ones(1,7), 9*ones(1,8), 10*ones(1,9),11*ones(1,10),12*ones(1,11)...
    13*ones(1,12), 14*ones(1,12), 15*ones(1,12), 16*ones(1,12);
    1, 1:2, 1:3, 1:4, 1:5, 1:6, 1:7, 1:8,  1:9,1:10,1:11, 1:12, 1:12, 1:12, 1:12]';

lengthMeasureIndices([1 6 15 28 45 66],:) = []; %eliminate bar measures
superBallCommandPlot = TensegrityPlot(nodes, strings, bars, 0.025,0.005);
N = 5;

superBallDynamicsPlot = TensegrityPlot(nodes, strings, bars, 0.025,0.005);
superBall = TensegrityStructure(nodes, strings, bars, F, stringStiffness,...
    barStiffness, stringDamping,barDamping, nodalMass, delT,delTUKF,stringRestLength,gravity);
superBall.baseStationPoints = baseStationPoints;
f = figure('units','normalized','outerposition',[0 0 1 1]);

%%%%%% Dynamics Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ax2 = axes;%subplot(1,2,2,'Parent',f,'units','normalized','outerposition',...
%[0.51 0.1 0.48 0.9]);
colormap([0.8 0.8 1; 0 1 1])
% use a method within TensegrityPlot class to generate a plot of the
% structure
generatePlot(superBallDynamicsPlot,ax2);
updatePlot(superBallDynamicsPlot);
scatter3(baseStationPoints(:,1),baseStationPoints(:,2),baseStationPoints(:,3),'fill','m');

groundTruthLines_test3 = [
    % Init Position
    [1.09   1.60    0]; % 9
    [1.80	0.92	0]; % 6
    [2.09	1.86	0]; % 8
    % Flop 1
    [2.16	1.96	0]; % 8'
    [3.29	1.90	0]; % 10
    [2.71	1.07	0]; % 12
    % Flop 2
    [2.77	1.14	0]; % 12'
    [3.58   0.43    0]; % 7
    [3.80   1.40    0]; % 4
%     % Flop 3
%     [3.90   1.46    0]; % 4'
%     [4.93   1.20    0]; % 11
%     [4.34   0.44    0]; % 2
    
];
groundTruthLables_test3 = {'9' '6' '8' '' '10' '12' '' '7' '4'};% '4' '11' '2'};

scatter3(groundTruthLines_test3(:,1),groundTruthLines_test3(:,2),groundTruthLines_test3(:,3),'fill','o');

labels = [labels groundTruthLables_test3];
hh = text([nodes(:,1);baseStationPoints(:,1);groundTruthLines_test3(:,1)],[nodes(:,2);baseStationPoints(:,2);groundTruthLines_test3(:,2)],[nodes(:,3);baseStationPoints(:,3);groundTruthLines_test3(:,3)]+0.1,labels,'FontSize',10);

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

textPositions = zeros(48,3);

for i =1:48
    textPositions(i,:) = (lengthMeasures(2*i-1,:) + lengthMeasures(2*i,:)*3)/4;
end
for i =1:4
lines(i) = plot3(lengthMeasures(24*i+(-23:0),1),lengthMeasures(24*i+(-23:0),2),lengthMeasures(24*i+(-23:0),3));
end
text1 = cellstr(num2str(zeros(48,1),2));
% hh2 = text(textPositions(:,1),textPositions(:,2),textPositions(:,3),text1,'FontSize',8);
% hh = [hh;hh2];


%settings to make it pretty
axis equal
view(3)
grid on
light('Position',[0 0 10]);%,'Style','local')
% lighting flat
xlim([-lims lims])
ylim([-lims lims])
zlim(0.8*[-0.01 lims])
title('UKF Output');
xlabel('X')
ylabel('Y')
zlabel('Z')
ax1  = ax2;

%%%%%%%%%%%%%%%% Make Video %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create a new VideoWriter object (an empty video file). Use whatever format you want,
% but in my experience MP4 (with H.264 codec) is by far the best. Please stop using AVI.
hvid = VideoWriter('./movie.mp4','MPEG-4');

% Full quality, because why not?
set(hvid,'Quality',100);

% Set the frame rate
set(hvid,'FrameRate',30);

% Open the object for writing
open(hvid);

% Desired frame resolution (see fig2frame). The video will automatically adopt the resolution of the first frame (see HELP VIDEOWRITER).
% You could instead set the Width property of the video object, but I prefer this.
framepar.resolution = [1024,768];

% Create a new figure
% f is already a figure obj
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

superBallUpdate(superBall,superBallDynamicsPlot,tspan,[ax1 ax2],hh,barLength,lines);
open(movieFrames);
rosMessageListener = rossubscriber('/ranging_data_matlab','std_msgs/Float32MultiArray',@(src,msg) superBallUpdate(double(msg.Data)));
lh = addlistener(f,'ObjectBeingDestroyed',@(f1,f2) clearThing(rosMessageListener));



