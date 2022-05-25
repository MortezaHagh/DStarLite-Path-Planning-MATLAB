function [Open, RHS, newobstNode, Model] = checkForUpdate(Open, RHS, newobstNode, Model, G, t, Start)

% check for Map change
if t==4
    
    xyStart = Start.cord;
    xySlast = Model.sLast.cord;
    Model.km = Model.km +Distance(xySlast(1), xySlast(2), xyStart(1), xyStart(2), Model.distType);
    Model.sLast = Start;
    
    % update model
    newobstNode = [newobstNode 36];
    Model.Obst.nodeNumber(end+1) = 36;
    Model.Obst.count = Model.Obst.count+1;
    
    neighbors = Model.Successors{newobstNode};
    for neighbor = neighbors
        costOld = Model.cost(neighbor, newobstNode(end));
        Model.cost(neighbor, newobstNode(end)) = 1000;
        Model.cost(newobstNode(end), neighbor) = 1000;
        if costOld > Model.cost(neighbor, newobstNode(end))
            if (neighbor~=Model.Robot.targetNode)
                RHS(neighbor) = min(RHS(neighbor), Model.cost(neighbor, newobstNode(end))+G(newobstNode(end)));
            end
        elseif (RHS(neighbor)==costOld+G(newobstNode(end)))
            if (neighbor~=Model.Robot.targetNode)
                succNodes = Model.Successors{neighbor};
                RHS(neighbor) = min(G(succNodes) + Model.cost(neighbor, succNodes));
            end
        end
        
    end
    
    % update vertex
    nodesForUpdate = newobstNode(end);
    [Open, RHS] = updateVertex(Open, RHS, G, nodesForUpdate, Model, Start);
    
end

end
