%Ѱ������Ľڵ�
function [Node_nearest_index, Dist_min] = Nearest(Tree, Node_rand, Node_added_index, Weight)
    Node_added = Tree.Node(:, 1:Node_added_index - 1);
    %�Ѿ���ӵĽڵ�
    
    Dist = Distance(Node_added, Node_rand, Weight);
    %��������ڵ㵽�Ѿ���ӵĽڵ��ľ���
    
	[Dist_min, Node_nearest_index] = min( Dist );
    %�ڲ����ռ��ڣ�����ŷ����þ���������ĵ㡣
    
end