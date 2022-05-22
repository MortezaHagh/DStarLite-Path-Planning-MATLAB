function path_nodes = finalPathNodes(G, model)

targetNode = model.targetNode;
startNode = model.startNode;
node = startNode;
path_nodes = [];
i=1;

path_nodes(i) = startNode;
while node ~= targetNode
    i = i+1;
    suc_nodes = adjacentNodes(node, model);
    [~, ind_minG] = min(G(suc_nodes)+ model.cost(node, suc_nodes));
    node = suc_nodes(ind_minG);
    path_nodes(i) = node;
end

% path_nodes = flip(path_nodes);

end