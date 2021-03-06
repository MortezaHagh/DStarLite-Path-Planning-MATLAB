function Model=createModelBase(Model)
% Create Complete Model - Base function for Astar

disp('Create Complete Model - Astar');

%% Map Size
Map.lim = 28;
Map.xMin = -1;
Map.xMax = Map.lim;
Map.yMin = -1;
Map.yMax = Map.lim;

Map.nX=Map.xMax-Map.xMin+1;
Map.nY=Map.yMax-Map.yMin+1;

%% robot data

% dir: direction
Robot.dir = 90; %randsample([0 90 180 270], 1);

% start & goal - start & target coordinates
Robot.xs = 0;
Robot.ys = 0;
Robot.xt = 6;
Robot.yt = 7;

%  start & goal - node numbers
Robot.startNode = (Robot.ys-Map.yMin)*(Map.xMax-Map.xMin+1)+Robot.xs+abs(Map.xMin-1);
Robot.targetNode = (Robot.yt-Map.yMin)*(Map.xMax-Map.xMin+1)+Robot.xt+abs(Map.xMin-1);

%% Obstacles

% radius
Obst.r = 0.25;

% Obstacle coordinates
xc1=[3 3 3 5 5 5 7 7 7 9 9 9 11 11 11];
yc1=[3 4 5 3 4 5 3 4 5 3 4 5 3 4 5];

xc1 = xc1+1;
yc1 = yc1+1;

xc2=[xc1 xc1 xc1 xc1];
yc2=[yc1 yc1+6 yc1+11 yc1+16];

xc3=xc2+12;
yc3=yc2;

Obst.x=[xc2 xc3];
Obst.y=[yc2 yc3];

Obst.count = length(Obst.x);

% obstacle node numbers
Obst.number = zeros(1,Obst.count);
for ix = 1:Obst.count
    Obst.nodeNumber(ix) = (Obst.y(ix)-Map.yMin)*(Map.xMax-Map.xMin+1)+Obst.x(ix)+abs(Map.xMin-1);
end

%% nodes & adj data
iNode = 0;
for iy = Map.yMin:Map.yMax
    for ix = Map.xMin:Map.xMax
        iNode = iNode+1;
        Nodes.number(1, iNode) = iNode;   % node number
        Nodes.cord(1:2, iNode) = [ix, iy]'; % node coordinates
    end
end
Nodes.count = iNode;

%% save model
Model.Nodes = Nodes;
Model.Robot = Robot;
Model.Obst = Obst;
Model.Map = Map;

%% plot model
% plotModel(model);

end