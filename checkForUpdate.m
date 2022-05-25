function [Open, RHS, newobstNode, Model] = checkForUpdate(Open, RHS, newobstNode, Model, G, t, Start)

% check for Map change
if t==4
    % update model
    newobstNode = [newobstNode 36];
    Model.Obst.nodeNumber(end+1) = 36;
    Model.Obst.count = Model.Obst.count+1;
    Model.cost(:, newobstNode(end)) = 1000;
    Model.cost(newobstNode(end, :)) = 1000;
    
    xyStart = Start.cord;
    xySlast = Model.sLast.cord;
    Model.km = Model.km +Distance(xySlast(1), xySlast(2), xyStart(1), xyStart(2), Model.distType);
    Model.sLast = Start;
    
    % update vertex
    nodesForUpdate = newobstNode(end);
    [Open, RHS] = updateVertex(Open, RHS, G, nodesForUpdate, Model);
    
end

end