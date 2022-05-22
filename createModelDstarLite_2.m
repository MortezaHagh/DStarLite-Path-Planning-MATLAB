function model=createModelDstarLite_2(model)

disp('Create Model');

%% robot
% dir: direction , r,l,u,d
dir = int32('d');

% start & goal
xs = 7;
ys = 2;
xt = 7;
yt = 7;

%% Area
limArea = 15;
xmin = -1;
xmax = limArea;
ymin = -1;
ymax = limArea;

x_node_num=xmax;
y_node_num=ymax;

% Obstacle
r = 0.25;
xc=[3 4 5 6 7 8 9 10 11 6 6 6 9 9];
yc=[3 3 3 3 3 3 3  3 3 2 1 0 2 1];

%% nodes & adj
k = 0;
adj = cell(1,1);
for j = ymin:ymax
    for i = xmin:xmax
        k = k+1;
        adj{k,1} = k;                   % node number
        adj{k,2} = [i, j];              % node coordinates
        nodes.number(1, k) = k;         % node number
        nodes.cord(1:2, k) = [i, j]';   % node coordinates
        
        if i == xs && j == ys
            start_node = k;             % start node number
        elseif i == xt && j == yt
            target_node = k;            % target (final) node number
        end
    end
end

% obstacle node numbers
obstNode = zeros(1,length(xc));
for i = 1:length(xc)
    for j = 1:size(nodes.number,2)
        if xc(i) == nodes.cord(1,j) && yc(i) == nodes.cord(2,j)
            obstNode(i) = nodes.number(j);
        end
    end
end

%% edge costs, G, RHS
if strcmp(model.adj_type,'4adj')
    q = [1 0; 0 1; 0 -1; -1 0];
    n_adj=4;
elseif strcmp(model.adj_type,'8adj')
    q = [1 0; 0 1; 0 -1; -1 0; 1 1; -1 -1; 1 -1; -1 1];
    n_adj=8;
end

nodes_count = k;
successors = cell(nodes_count,1);
cost = inf*ones(nodes_count, nodes_count);

for node=1:nodes_count
    xNode = nodes.cord(1,node);
    yNode = nodes.cord(2,node);
    for k=1:n_adj
        i=q(k,1);  j=q(k,2);
        s_x = xNode+i;
        s_y = yNode+j;
        % check if the Node is within array bound
        if((s_x>=xmin && s_x<=xmax) && (s_y>=ymin && s_y<=ymax))
            s_node = node+i+(j*(ymax-ymin+1));
            successors{node} = [successors{node}, s_node];
            if ~any(s_node==obstNode) && ~any(node==obstNode)
                cost(node, s_node) = 1;
                cost(s_node, node) = 1;
            end
        end
    end
end

% G, RHS
G = inf(1, nodes_count);
RHS = inf(1, nodes_count);

%% save model
model.x_node_num = x_node_num;
model.y_node_num = y_node_num;
model.targetNode = target_node;
model.startNode = start_node;
model.obstNode = obstNode;
model.numOfObs=numel(xc);
model.limArea = limArea;
model.nodes = nodes;
model.xmin = xmin;
model.xmax = xmax;
model.ymin = ymin;
model.ymax = ymax;
model.obst_r = r;
model.dir = dir;
model.adj = adj;
model.xs = xs;
model.ys = ys;
model.xt = xt;
model.yt = yt;
model.xc = xc;
model.yc = yc;

model.successors=successors;
model.cost=cost;
model.RHS=RHS;
model.G=G;


%% plot model
% plotModel(model);

end
