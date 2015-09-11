
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

options = optimoptions('quadprog','Algorithm',  'interior-point-convex','Display','off');

baseStationPoints = [
     5.3500    1.2500    0.3500;
     2.7200    0.6200    2.6600;
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

HH  = makehgtform('axisrotate',[0 1 0],0.6);
HH  = makehgtform('axisrotate',[1 0 0],0.6)*HH;
nodes = (HH(1:3,1:3)*nodes')';
nodes(:,3) = nodes(:,3) - min(nodes(:,3));
nodes(:,2) = nodes(:,2) - 0.95 ;
nodes(:,1) = nodes(:,1) - 0.95 ;

bars = [1:2:11;
    2:2:12];
strings = [1  2 3 4 5 6 7 8  9 11 12  10 1 1 11 11 10 10 3 3 7  7 6 6;
           11 5 7 2 9 3 6 12 8 10 4   1 9 5  2  4  12  8 5 2 4 12 8 9];

stringRestLength = [(1-(preTension/Ka))*ones(12,1)*norm(nodes(2,:)-nodes(5,:)); %active
                                                              0.865*ones(12,1)]; %passive

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
hh = text([nodes(:,1);baseStationPoints(:,1)],[nodes(:,2);baseStationPoints(:,2)],[nodes(:,3);baseStationPoints(:,3)]+0.1,labels,'FontSize',10);

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
hh2 = text(textPositions(:,1),textPositions(:,2),textPositions(:,3),text1,'FontSize',8);
hh = [hh;hh2];


%settings to make it pretty
axis equal
view(3)
grid on
light('Position',[0 0 10]);%,'Style','local')
lighting flat
xlim([-lims lims])
ylim([-lims lims])
zlim(0.8*[-0.01 lims])
title('UKF Output');
xlabel('X')
ylabel('Y')
zlabel('Z')
ax1  = ax2;
superBallUpdate(superBall,superBallDynamicsPlot,tspan,[ax1 ax2],hh,barLength,lines);
rosMessageListener = rossubscriber('/ranging_data_matlab','std_msgs/Float32MultiArray',@(src,msg) superBallUpdate(double(msg.Data)));
lh = addlistener(f,'ObjectBeingDestroyed',@(f1,f2) clearThing(rosMessageListener));




