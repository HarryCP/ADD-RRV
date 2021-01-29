%寻找最近的节点
function [Node_nearest_index, Dist_min] = Nearest(Tree, Node_from, Node_added_index)
    Node_add = Tree.Node(:, 1:Node_added_index - 1);
    %已经添加的节点
    
    Dist = Distance(Node_from, Node_add);
    %计算随机节点到已经添加的节点间的距离
    
	[Dist_min, Node_nearest_index] = min(Dist);
    %在采样空间内，根据欧几里得距离求最近的点。
    
end