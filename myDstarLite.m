function [Model, Path] = myDstarLite(Model)
% DstarLite algorithm

% initialization
[G, RHS, open, Model] = initializeDstarLite(Model);

t=1;
newobstNode=[];
Path.nodeNumbers = [Model.Robot.startNode];

% update start_key
Start.nodeNumber = Model.Robot.startNode;
Start.key = min(G(Start.nodeNumber), RHS(Start.nodeNumber))*[1; 1];
Start.cord = nodes2coords(Start.nodeNumber, Model);
currentDir = deg2rad(Model.Robot.dir);
Model.sLast = Start;

% compute shortest path
[G, RHS, open, Start] = computeShortestPath(G, RHS, open, Start, Model);

%% main procedure

% if G(Start.nodeNumber)=inf -> then there is no known path

while Start.nodeNumber~=Model.Robot.targetNode
    
    % move robot to next node
    sucNodes = Model.Successors{Start.nodeNumber};
    switch Model.expandMethod
        case 'heading'
            dTheta = turnCost(Start.nodeNumber, sucNodes, Model, currentDir);
            [~, sortInds] = sortrows([G(sucNodes) + Model.cost(Start.nodeNumber, sucNodes); abs(dTheta)]');
            Start.nodeNumber = sucNodes(sortInds(1));
            currentDir = currentDir + dTheta(sortInds(1));
        case 'random'
            [~, minInd] = min(G(sucNodes) + Model.cost(Start.nodeNumber, sucNodes));
            Start.nodeNumber = sucNodes(minInd);
    end
    Start.coords = nodes2coords(Start.nodeNumber, Model);
    
    % move to Start.nodeNumber and add Start.nodeNumber to Path
    Path.nodeNumbers(end+1) = Start.nodeNumber;
    t=t+1;
    
    % check for update in edge costs (obstacles)
    [open, RHS, newobstNode, Model] = checkForUpdate(open, RHS, newobstNode, Model, G, t, Start);
    
    % compute shortest path
    [G, RHS, open, Start] = computeShortestPath(G, RHS, open, Start, Model);
    
end

%% optimal paths coordinations, nodes, directions
Path.coords = nodes2coords(Path.nodeNumbers, Model);
Path.dirs = nodes2dirs(Path.nodeNumbers, Model);

% update model
newObstXY = Model.Nodes.cord(:,newobstNode);
Model.Obst.x = [Model.Obst.x, newObstXY(1,:)];
Model.Obst.y = [Model.Obst.y, newObstXY(2,:)];

end
