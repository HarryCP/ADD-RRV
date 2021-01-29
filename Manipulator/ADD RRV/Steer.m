%��������״̬
function [State, End, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
            Steer(Tree, From_index, State_to, Step, Boundary, RobotData, ObstacleData, Robot_parameter, Num_collision_test, Weight, Error_goal)
    
    State_from = Tree.State(:, From_index);
    
    State = [];
    End = [];
    Cost = [];
    State_collision = [];
    
    Dist_total = Distance(State_from, State_to, Weight, 1);         % �������ܳ���
    Vector = Weight*(State_to - State_from);                        % ��չ����
    Vect_step = Vector * (Step/Dist_total);                         % ��չ�����ϵĲ���

    if Dist_total > Step
        State_new =  State_from + Vect_step;
        Type_steer = 1;
        %ǰ��һ��
    else
        State_new = State_to;
        Type_steer = 2;
        %����Ŀ�������                
    end
    
    Judge = IsCollision(State_new, Boundary, RobotData, ObstacleData, Robot_parameter);
	% ��ײ���
    Num_collision_test = Num_collision_test + 1;
    
    if Judge == 0 %δ������ײ
        State = State_new;
        End = ForwardKine(State_new, Robot_parameter);
        Cost = Distance(State_new, State_from, Weight, 1);
    else
        State_collision = State_new;
        Type_steer = 3;
    end
    
end