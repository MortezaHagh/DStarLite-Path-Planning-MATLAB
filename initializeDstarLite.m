function [G, RHS, Open, Model] = initializeDstarLite(Model)
% Initialize DstarLite PP problem.


% DstarLite parameters
Model.km=0;

% G, RHS
G = Model.G;
RHS = Model.RHS;

% Open: count, List
% node hCost, key, ind

% set the target node as the first node in Open
TopNode.nodeNumber=Model.Robot.targetNode;
TopNode.hCost = Distance(Model.Robot.xs, Model.Robot.ys, Model.Robot.xt, Model.Robot.yt, Model.distType);
TopNode.key = [TopNode.hCost; 0];
RHS(TopNode.nodeNumber)=0;
TopNode.ind = 1;

% insert start node in open list
Open.List(1) = TopNode;
Open.count = 1;

end