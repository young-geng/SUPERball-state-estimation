
clc 
clear variables 
close all

barSpacing = 0.375;
barLength = 1.7;
lims = 1.2*barLength;

tspan =0.1;          % time between plot updates in seconds
delT = 0.001;         % timestep for dynamic sim in seconds
delTUKF  = 0.01;
K = 998;              %outer rim string stiffness in Newtons/meter
nodalMass = [0; 1.625];
c = 40;             % damping constant, too lazy to figure out units.
F = zeros(2,3);
stringStiffness = K;
barStiffness = [];
stringDamping = c;% c*ones(24,1);  %string damping vector

options = optimoptions('quadprog','Algorithm',  'interior-point-convex','Display','off');

LL = 2.4-0.73; %meter
nodes = [-0.96+0.96/2         ,   1.15-1.15/2 ,  2.4;
         -0.96+0.96/2+LL*sin(0) , 1.15-1.15/2 ,  2.4-LL*cos(0)];
     baseStationPoints = [0+0.96/2     ,   0-1.15/2      ,  1.63;
                         -1.362+0.96/2  ,   0-1.15/2      ,  1.6606 ;  
                         -2.4712+0.96/2  ,  1.1885-1.15/2 ,  1.9514;  
                          0.2882+0.96/2  ,  2.4010-1.15/2  ,  1.8013;  
                         -1.0626+0.96/2   , 2.4519-1.15/2 ,  1.7435 ];  
stationLengths = sqrt(sum((baseStationPoints - repmat(nodes(2,:),5,1)).^2,2));
disp(stationLengths)
     
strings = [1; 
           2];
    
bars = [];
stringRestLength = LL;

lengthMeasureIndices = [2*ones(1,5);
                        3:7]';
pendulumCommandPlot = TensegrityPlot(nodes, strings, bars, 0.0375,0.0075);

pendulumPlot = TensegrityPlot([nodes; baseStationPoints], strings, bars, 0.0375,0.0075);
pendulum = TensegrityStructure(nodes, strings, bars, F, stringStiffness,...
    barStiffness, stringDamping, nodalMass, delT,delTUKF,stringRestLength);
pendulum.baseStationPoints = baseStationPoints;

f = figure('units','normalized','outerposition',[0 0 1 1]);

%%%%%%%% IK Subplot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ax = subplot(1,2,1,'Parent',f,'units','normalized','outerposition',...
    [0.01 0.1 0.48 0.9]);

% use a method within TensegrityPlot class to generate a plot of the
% structure
generatePlot(pendulumCommandPlot,ax)
updatePlot(pendulumCommandPlot);
%settings to make it pretty
axis equal
view(3)
grid on
light('Position',[0 0 1],'Style','local')
%lighting flat
lighting gouraud
colormap cool% winter
xlim([-lims lims])
ylim([-lims lims])
zlim(1.6*[-0.01 lims])
title('True Dynamics');


%%%%%% Dynamics Subplot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ax1 = subplot(1,2,2,'Parent',f,'units','normalized','outerposition',...
    [0.51 0.1 0.48 0.9]);

% use a method within TensegrityPlot class to generate a plot of the
% structure
generatePlot(pendulumPlot,ax1);
updatePlot(pendulumPlot);
linkprop([ax ax1], 'CameraPosition');
x = (lims-0.001)*[-1 1 1 -1];
y = (lims-0.001)*[-1 -1 1 1];
patch(x,y,zeros(size(x)))
%settings to make it pretty
axis equal
view(3)
grid on
light('Position',[0 0 1],'Style','local')
lighting flat
xlim([-lims lims])
ylim([-lims lims])
zlim(1.6*[-0.01 lims])
title('UKF outPuts');

% 
%for i = 1:600
%    pendulumUpdate
%end

% 
% posesub = rossubscriber('/bbb2/0x2017_0x1', 'superball_msg/TimestampedFloat32',@(src,msg) disp(msg.Data))
%  
% 
% 
% /bbb2/0x2717_0xa
% /bbb2/0x2017_0xa
% /bbb4/0x2017_0xa
% /bbb8/0x2717_0xa
% /bbb8/0x2017_0xa
% 

calls = ROSCallbackFunctions( [stationLengths zeros(5,1)]);
%lengthListener = rossubscriber('/bbb2/0x2717_0xa', 'std_msgs/Float32MultiArray',@(src,msg) updateVal(calls,msg,1));
lengthListener = rossubscriber('/ranging_data_matlab', 'std_msgs/Float32MultiArray',@(src,msg) updateVal(calls,msg));
% lengthListener2 = rossubscriber('/bbb2/0x2017_0xa', 'superball_msg/TimestampedFloat32',@(src,msg) updateVal(calls,msg,2));
% lengthListener3 = rossubscriber('/bbb4/0x2017_0xa', 'superball_msg/TimestampedFloat32',@(src,msg) updateVal(calls,msg,3));
% lengthListener4 = rossubscriber('/bbb8/0x2717_0xa', 'superball_msg/TimestampedFloat32',@(src,msg) updateVal(calls,msg,4));
% lengthListener5 = rossubscriber('/bbb8/0x2017_0xa', 'superball_msg/TimestampedFloat32',@(src,msg) updateVal(calls,msg,5));
% 
pause(0.5)
disp(calls.vals)
pendulumUpdate([stationLengths ones(5,1)] ,pendulum,pendulumCommandPlot,pendulumPlot,tspan);
% 
 t = timer;
 funcHandle = @(vec) pendulumUpdate(vec);
% for i =1:100
%     timerUpdate(calls,funcHandle)
% end
% 
t.TimerFcn = @(myTimerObj, thisEvent) timerUpdate(calls,funcHandle);
t.Period = tspan;
t.ExecutionMode = 'fixedRate';
start(t);
% % 
% % 
% 

