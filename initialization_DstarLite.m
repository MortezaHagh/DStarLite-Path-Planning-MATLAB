function [G, RHS, open, model] = initialization_DstarLite(model)

% DstarLite parameters
model.km=0;

% G, RHS
G = model.G;
RHS = model.RHS;

% Open list
% open.count
% open.list.
% node cost_h key ind

% set the target node as the first node in Open
topnode.node=model.targetNode;
topnode.cost_h = Distance(model.xs, model.ys, model.xt, model.yt, model.dist_type);
topnode.key = [topnode.cost_h; 0];
RHS(topnode.node)=0;
topnode.ind = 1;

% insert start node in open list
open.list(1) = topnode;
open.count = 1;

end