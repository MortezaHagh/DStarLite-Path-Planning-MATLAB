function [G, RHS, Open, Start] = computeShortestPath(G, RHS, Open, Start, Model)
% computeShortestPath between current startNode and targetNode

% select top key
TopNode = topKey(Open);

% update start_key
Start.key = min(G(Start.nodeNumber), RHS(Start.nodeNumber))*[1; 1]+[Model.km; 0];

while compareKeys(TopNode.key, Start.key) || RHS(Start.nodeNumber)>G(Start.nodeNumber)
    
    k_old = TopNode.key;
    k_new = min(G(TopNode.nodeNumber), RHS(TopNode.nodeNumber)) + [TopNode.hCost+Model.km; 0];
    
    % update vertex
    if compareKeys(k_old, k_new)
        Open.List(TopNode.ind).key = k_new;
    else
        if G(TopNode.nodeNumber)>RHS(TopNode.nodeNumber)
            G(TopNode.nodeNumber) = RHS(TopNode.nodeNumber);
            Open.List(TopNode.ind)=[];
            Open.count = Open.count-1;
            for predNode=Model.Successors{TopNode.nodeNumber}
                if predNode~=Model.Robot.targetNode
                    RHS(predNode) = min(RHS(predNode), Model.cost(predNode, TopNode.nodeNumber)+G(TopNode.nodeNumber));
                end
                nodesForUpdate = [predNode];
                [Open, RHS] = updateVertex(Open, RHS, G, nodesForUpdate, Model, Start);
            end
        else
            gOld = G(TopNode.nodeNumber);
            G(TopNode.nodeNumber) = inf;
            for nodeNumber = [Model.Successors{TopNode.nodeNumber} TopNode.nodeNumber]
                if (RHS(nodeNumber)==Model.cost(nodeNumber, TopNode.nodeNumber) + gOld)
                    if nodeNumber~=Model.Robot.targetNode
                        succNodes = Model.Successors{nodeNumber};
                        RHS(nodeNumber) = min(G(succNodes) + Model.cost(nodeNumber, succNodes));
                    end
                end
                nodesForUpdate = [nodeNumber];
                [Open, RHS] = updateVertex(Open, RHS, G, nodesForUpdate, Model, Start);
            end
        end
    end
    
    % select top key
    TopNode = topKey(Open);
    
    % update goal_key
    Start.key = min(G(Start.nodeNumber), RHS(Start.nodeNumber))*[1; 1]+[Model.km; 0];
    
end

end