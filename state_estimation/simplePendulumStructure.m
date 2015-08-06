
clc 
clear variables 
close all

barSpacing = 0.375;
barLength = 1.7;
lims = 1.2*barLength;

tspan =0.05;          % time between plot updates in seconds
delT = 0.001;         % timestep for dynamic sim in seconds
delTUKF  = 0.005;
K = 998;              %outer rim string stiffness in Newtons/meter
nodalMass = [0; 1.625];
c = 400;             % damping constant, too lazy to figure out units.
F = zeros(2,3);
stringStiffness = K;
barStiffness = [];
stringDamping = c;% c*ones(24,1);  %string damping vector

options = optimoptions('quadprog','Algorithm',  'interior-point-convex','Display','off');

LL = 1; %meter
nodes = [0          0    3/2*LL;
         LL/sqrt(2)  0   3/2*LL-LL/sqrt(2)];
     
strings = [1; 
           2];
    
bars = [];
stringRestLength = LL;

lengthMeasureIndices = [2*ones(1,5);
                        3:7]';
pendulumCommandPlot = TensegrityPlot(nodes, strings, bars, 0.025,0.005);

pendulumPlot = TensegrityPlot(nodes, strings, bars, 0.025,0.005);
pendulum = TensegrityStructure(nodes, strings, bars, F, stringStiffness,...
    barStiffness, stringDamping, nodalMass, delT,delTUKF,stringRestLength);

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
pendulumUpdate(pendulum,pendulumCommandPlot,pendulumPlot,tspan);
% 
%for i = 1:600
%    pendulumUpdate
%end
% 
t = timer;
t.TimerFcn = @(myTimerObj, thisEvent) pendulumUpdate;
t.Period = tspan;
t.ExecutionMode = 'fixedRate';
start(t);
% 
% 


