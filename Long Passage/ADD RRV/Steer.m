%连接两个状态
function [Node_new, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                Steer(Tree, From_index, Node_to, Step, Robot_size, Boundary, Obstacle, Num_collision_test, Weight)  
	%Type_steer = 1, 前进一步
    %Type_steer = 2，到达采样点
    %Type_steer = 3, 发生碰撞
    
    Node_from = Tree.Node(:, From_index);
    
    if AttitudeDifference(Node_from(3, :), Node_to(3, :)) < abs(Node_from(3, :) - Node_to(3, :))
        if Node_from(3, :) < 0
            Node_to(3, :) = Node_to(3, :) - 2*pi;
        else
            Node_to(3, :) = Node_to(3, :) + 2*pi;
        end
	end
    
    Node_new = [];
    Cost = [];
    State_collision = [];
    
    Dist_total = Distance(Node_from, Node_to, Weight);      % 两点间的总长度
    Vect = Weight*(Node_to - Node_from);                    % 拓展方向 
    Step_vect = Vect*(Step/Dist_total);                     % 拓展方向上的步长
       
    if Dist_total > Step
        State_new = Node_from + Step_vect;
        State_new(3, :) = GetAttiFromTrans(RotZ(State_new(3, :)));  %对末端姿态进行校正
        Type_steer = 1;
        %前进一步
    else
        State_new = Node_to;
        State_new(3, :) = GetAttiFromTrans(RotZ(State_new(3, :)));  %对末端姿态进行校正
        Type_steer = 2;
        %到达目标采样点
    end
    
    Judge = IsCollision(State_new, Robot_size, Boundary, Obstacle);
	% 碰撞检测
	Num_collision_test = Num_collision_test + 1;    
           
    if Judge == 0 %未发生碰撞
        Node_new = State_new;
        Cost = Distance(Node_from, Node_new, Weight);
    else        %发生碰撞
        State_collision = State_new;
        Type_steer = 3;
    end
    
    
    
    
    
end