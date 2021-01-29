%连接两个状态
function [Node_new, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
            Steer(Tree, From_index, Node_to, Step, Robot_size, Boundary, Obstacle, Num_collision_test)
    %Type_steer = 1, 前进一步
    %Type_steer = 2，到达采样点
    %Type_steer = 3, 发生碰撞
        
        
    Node_from = Tree.Node(:, From_index);
    
    Node_new = [];
    Cost = [];
    State_collision = [];
    
    Dist_total = Distance(Node_from, Node_to);      % 两点间的总长度
    Vect = Node_to - Node_from;                     % 拓展方向                      
    Step_vect = Vect*(Step/Dist_total);              % 拓展方向上的步长
    
    if Dist_total > Step
        State_new = Node_from + Step_vect;
        Type_steer = 1;
    else
        State_new = Node_to;
        Type_steer = 2;
    end
    
    Judge = IsCollision(State_new, Robot_size, Boundary, Obstacle);
	% 碰撞检测
	Num_collision_test = Num_collision_test + 1;
    
    if Judge == 0 %未发生碰撞
        Node_new = State_new;
        Cost = Distance(Node_from, Node_new);
    else        %发生碰撞
        State_collision = State_new;
        Type_steer = 3;
    end

    
end