function [model, path] = myDstarLite(model)


% initialization
[G, RHS, open, model] = initialization_DstarLite(model);

t=1;
newobstNode=[];
path.nodes = [model.startNode];

% update start_key
start.node = model.startNode;
start.key = min(G(start.node), RHS(start.node))*[1; 1];
model.s_last = start.node;

% compute shortest path
[G, RHS, open, start] = computeShortestPath(G, RHS, open, start, model);

%% main procedure

% if G(s_start)=inf -> then there is no known path

while start.node~=model.targetNode
    
    % move robot to next node (start)
    suc_nodes = model.successors{start.node};
    [~, min_ind] = min(G(suc_nodes) + model.cost(start.node, suc_nodes));
    start.node = suc_nodes(min_ind);
    
    % move to s_start and add s_start to path
    path.nodes(end+1) = start.node;
    t=t+1;
    
    % check for update in edge costs (obstacles)
    [open, RHS, newobstNode, model] = checkForUpdate(open, RHS, newobstNode, model, G, t, start.node);
    
    % compute shortest path
    [G, RHS, open, start] = computeShortestPath(G, RHS, open, start, model);
    
end

%% optimal paths coordinations, nodes, directions
path.coords = nodes2coords(path.nodes, model);
path.dirs = nodes2dirs(path.nodes, model);

% update model
new_obst_xy = model.nodes.cord(:,newobstNode);
model.xc = [model.xc, new_obst_xy(1,:)];
model.yc = [model.yc, new_obst_xy(2,:)];

end
