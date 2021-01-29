%连接两个状态
function [State, End, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
            Steer(Tree, From_index, State_to, Step, Boundary, RobotData, ObstacleData, Robot_parameter, Num_collision_test, Weight, Error_goal)
    
    State_from = Tree.State(:, From_index);
    
    State = [];
    End = [];
    Cost = [];
    State_collision = [];
    
    Dist_total = Distance(State_from, State_to, Weight, 1);         % 两点间的总长度
    Vector = Weight*(State_to - State_from);                        % 拓展方向
    Vect_step = Vector * (Step/Dist_total);                         % 拓展方向上的步长

    if Dist_total > Step
        State_new =  State_from + Vect_step;
        Type_steer = 1;
        %前进一步
    else
        State_new = State_to;
        Type_steer = 2;
        %到达目标采样点                
    end
    
    Judge = IsCollision(State_new, Boundary, RobotData, ObstacleData, Robot_parameter);
	% 碰撞检测
    Num_collision_test = Num_collision_test + 1;
    
    if Judge == 0 %未发生碰撞
        State = State_new;
        End = ForwardKine(State_new, Robot_parameter);
        Cost = Distance(State_new, State_from, Weight, 1);
    else
        State_collision = State_new;
        Type_steer = 3;
    end
    
end