%寻找最近的节点
function [Node_nearest_index, Dist_min] = Nearest(Tree, End_rand, Node_added_index, Weight)
   
	Node_added = Tree.End(:, 1:Node_added_index - 1);
	%已经添加的节点
	Dist = Distance(Node_added, End_rand, Weight, 1);
	%计算随机构型空间节点到已经添加的节点间的距离
    
    [Dist_min, Node_nearest_index] = min( Dist );
    %在采样空间内，根据欧几里得距离求最近的点。
    
end