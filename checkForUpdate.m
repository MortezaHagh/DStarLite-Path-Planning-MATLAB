function [open, RHS, newobstNode, model] = checkForUpdate(open, RHS, newobstNode, model, G, t, s_start)

% check for update cost
if t==4
    % update model
    newobstNode = [newobstNode 36];
    model.obstNode(end+1) = 36;
    model.numOfObs = model.numOfObs+1;
    model.cost(:, newobstNode(end)) = 1000;
    model.cost(newobstNode(end, :)) = 1000;
    
    xy_ss = model.nodes.cord(:,s_start);
    xy_sl = model.nodes.cord(:,model.s_last);
    model.km = model.km +Distance(xy_sl(1), xy_sl(2), xy_ss(1), xy_ss(2), model.dist_type);
    model.s_last = s_start;
    
    % update vertex
    nodes_for_update = newobstNode(end);
    [open, RHS] = updateVertex(open, RHS, G, nodes_for_update, model);
    
end

end