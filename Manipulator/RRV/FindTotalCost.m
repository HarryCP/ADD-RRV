function TotalCost = FindTotalCost(Tree, Node_index)

    Node_num=length(Node_index);    %��Ҫ�����Ľڵ��������
    TotalCost = zeros(1, Node_num);
    
    for k = 1:Node_num 
        %�ҵ�Node_index����㵽���ڵ��ܴ���
        Current_index = Node_index(k);
        TotalCost(k) = Tree.Cost(1, Current_index);
        
         while Current_index ~= 1
            Current_index = Tree.Parent(Current_index);
            TotalCost(k) = TotalCost(k) + Tree.Cost(1, Current_index);
         end
        
    end
    
end