
clc 
clear all 
close all

barLength = 1.6;
totalSUPERballMass = 21;    % kg
barSpacing = barLength/4;
lims = 2*barLength;
gravity = 9.81;             % m/s^2
tspan =0.1;                % time between plot updates in seconds
delT = 0.001;               % timestep for dynamic sim in seconds
delTUKF  = 0.0025;
Kp = 998;                   %passive string stiffness in Newtons/meter
Ka = 3150;                  %active string stiffness in Newtons/meter
preTension = 100*(barLength/1.7);                   % how much force to apply to each cable in Newtons
nodalMass = (totalSUPERballMass/12)*ones(12,1);
Cp = 30;                    % damping constant, too lazy to figure out units.
Ca = 50;                    % constant for passive and active springs
barDamping = Cp/100*ones(6,1);
F = zeros(12,3);
stringStiffness = [Ka*ones(12,1); Kp*ones(12,1)];   % First set of 12 are acuated springs, second are passive
barStiffness = 100000*ones(6,1);
stringDamping = [Ca*ones(12,1); Cp*ones(12,1)];     % string damping vector

options = optimoptions('quadprog','Algorithm',  'interior-point-convex','Display','off');
 
baseStationPoints = [-2.3800   -0.9800    2.4300;
                     -3.3800         0    0.3500;
                           0         0    0.3500;
                     -0.7039   -2.1896    0.3500];
labels = {'1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16'};

                     
nodes = [barSpacing      0 -barLength*0.5;
         barSpacing      0  barLength*0.5;
         
          0  barLength*0.5              barSpacing;   %5
        0 -barLength*0.5               barSpacing;
         barLength*0.5               barSpacing      0;
         -barLength*0.5               barSpacing     0;      
        -barSpacing    0  barLength*0.5   ;
        -barSpacing    0  -barLength*0.5   ;             %8
        
        
         0  barLength*0.5              -barSpacing;
         0  -barLength*0.5             -barSpacing;
         barLength*0.5              -barSpacing     0;    
         -barLength*0.5              -barSpacing      0;
                                               ];
     
%HH  = makehgtform('axisrotate',[1 1 0],-0.3);
%    nodes = (HH(1:3,1:3)*nodes')';
%nodes(:,3) = nodes(:,3) +barLength*0.5 ;    
HH  = makehgtform('axisrotate',[0 1 0],0.6);
HH  = makehgtform('axisrotate',[1 0 0],0.6)*HH;
nodes = (HH(1:3,1:3)*nodes')';
nodes(:,3) = nodes(:,3) - min(nodes(:,3));  
nodes(:,2) = nodes(:,2) - 0.95 ;
nodes(:,1) = nodes(:,1) - 0.95 ;

bars = [1:2:11; 
        2:2:12];
strings = [2 5 9  8 12 4  1 11 10 3 7 6 1 1 11 11 10 10 3 3 7  7 6 6;
           5 9 8 12  4 2 11 10  1 7 6 3 9 5  2  4 12  8 5 2 4 12 8 9];
%strings = [1  1   1  1  2  2  2  2  3  3  3  3  4  4  4  4  5  5  6  6  7  7  8  8;
%           7  8  10 12  5  6 10 12  7  8  9 11  5  6  9 11 11 12  9 10 11 12  9 10];

stringRestLength = [(1-(preTension/Ka))*ones(12,1)*norm(nodes(2,:)-nodes(5,:)); %active
                    (1-(preTension/Kp))*ones(12,1)*norm(nodes(2,:)-nodes(5,:))]; %passive
%stringRestLength = 0.9*ones(24,1)*norm(nodes(1,:)-nodes(7,:));

lengthMeasureIndices = [2*ones(1,1), 3*ones(1,2), 4*ones(1,3), 5*ones(1,4), 6*ones(1,5), ...
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

%%%%%%%% IK Subplot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ax1 = subplot(1,2,1,'Parent',f,'units','normalized','outerposition',...
%     [0.01 0.1 0.48 0.9]);
% % use a method within TensegrityPlot class to generate a plot of the
% % structure
% generatePlot(superBallCommandPlot,ax1)
% updatePlot(superBallCommandPlot);
% %settings to make it pretty
% axis equal
% view(3)
% grid on
% light('Position',[0 0 10],'Style','local')
% %lighting flat
% lighting flat
% colormap([0.8 0.8 1; 0 1 1])
% xlim([-lims lims])
% ylim([-lims lims])
% zlim(1.6*[-0.01 lims])
% title('Simulated Input');
% 

%%%%%% Dynamics Subplot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ax2 = axes;%subplot(1,2,2,'Parent',f,'units','normalized','outerposition',...
    %[0.51 0.1 0.48 0.9]);
colormap([0.8 0.8 1; 0 1 1])
% use a method within TensegrityPlot class to generate a plot of the
% structure
generatePlot(superBallDynamicsPlot,ax2);
updatePlot(superBallDynamicsPlot);
scatter3(baseStationPoints(:,1),baseStationPoints(:,2),baseStationPoints(:,3),'fill','m');
hh = text([nodes(:,1);baseStationPoints(:,1)],[nodes(:,2);baseStationPoints(:,2)],[nodes(:,3);baseStationPoints(:,3)]+0.1,labels,'FontSize',6);

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
superBallUpdate(superBall,superBallDynamicsPlot,tspan,[ax1 ax2],hh,barLength);
rosMessageListener = rossubscriber('/ranging_data_matlab','std_msgs/Float32MultiArray',@(src,msg) superBallUpdate(double(msg.Data)));
lh = addlistener(f,'ObjectBeingDestroyed',@(f1,f2) clearThing(rosMessageListener));
% for i = 1:1000
%     superBallUpdate
%   %  MM(i) = getframe(f);
% end



