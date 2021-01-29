function TotalCost = FindTotalCost(Tree, Node_index)

    Node_num=length(Node_index);    %需要搜索的节点代价数量
    TotalCost = zeros(1, Node_num);
    
    for k = 1:Node_num 
        %找到Node_index这个点到根节点总代价
        Current_index = Node_index(k);
        TotalCost(k) = Tree.Cost(1, Current_index);
        
         while Current_index ~= 1
            Current_index = Tree.Parent(Current_index);
            TotalCost(k) = TotalCost(k) + Tree.Cost(1, Current_index);
         end
        
    end
    
end