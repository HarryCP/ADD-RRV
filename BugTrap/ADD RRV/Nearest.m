%Ѱ������Ľڵ�
function [Node_nearest_index, Dist_min] = Nearest(Tree, Node_from, Node_added_index)
    Node_add = Tree.Node(:, 1:Node_added_index - 1);
    %�Ѿ���ӵĽڵ�
    
    Dist = Distance(Node_from, Node_add);
    %��������ڵ㵽�Ѿ���ӵĽڵ��ľ���
    
	[Dist_min, Node_nearest_index] = min(Dist);
    %�ڲ����ռ��ڣ�����ŷ����þ���������ĵ㡣
    
end