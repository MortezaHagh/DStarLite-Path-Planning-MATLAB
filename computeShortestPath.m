function [G, RHS, open, start] = computeShortestPath(G, RHS, open, start, model)

% select top key
topnode = topKey(open);

% update start_key
start.key = min(G(start.node), RHS(start.node))*[1; 1];

while compareKeys(topnode.key, start.key) || RHS(start.node)~=G(start.node)
    
    k_old = topnode.key;
    k_new = min(G(topnode.node), RHS(topnode.node)) + [topnode.cost_h+model.km; 0];
    
    % remove topkey from open
    open.list(topnode.ind)=[];
    open.count = open.count-1;
    
    % update vertex
    nodes_for_update = model.successors{topnode.node};
    if compareKeys(k_old, k_new)
        open.list(end+1) = topnode;
        open.list(end+1).key = k_new;
    else
        if G(topnode.node)>RHS(topnode.node)
            G(topnode.node) = RHS(topnode.node);
        else
            G(topnode.node) = inf;
            nodes_for_update(end+1) = topnode.node;
        end
        [open, RHS] = updateVertex(open, RHS, G, nodes_for_update, model);
    end
    
    % select top key
    topnode = topKey(open);
    
    % update goal_key
    start.key = min(G(start.node), RHS(start.node))*[1; 1];
    
end

end