%Ѱ������Ľڵ�
function [Node_nearest_index, Dist_min] = Nearest(Tree, End_rand, Node_added_index, Weight)
   
	Node_added = Tree.End(:, 1:Node_added_index - 1);
	%�Ѿ���ӵĽڵ�
	Dist = Distance(Node_added, End_rand, Weight, 1);
	%����������Ϳռ�ڵ㵽�Ѿ���ӵĽڵ��ľ���
    
    [Dist_min, Node_nearest_index] = min( Dist );
    %�ڲ����ռ��ڣ�����ŷ����þ���������ĵ㡣
    
end