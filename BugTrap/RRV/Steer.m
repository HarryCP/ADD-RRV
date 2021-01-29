%��������״̬
function [Node_new, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
            Steer(Tree, From_index, Node_to, Step, Robot_size, Boundary, Obstacle, Num_collision_test)
    %Type_steer = 1, ǰ��һ��
    %Type_steer = 2�����������
    %Type_steer = 3, ������ײ
        
        
    Node_from = Tree.Node(:, From_index);
    
    Node_new = [];
    Cost = [];
    State_collision = [];
    
    Dist_total = Distance(Node_from, Node_to);      % �������ܳ���
    Vect = Node_to - Node_from;                     % ��չ����                      
    Step_vect = Vect*(Step/Dist_total);              % ��չ�����ϵĲ���
    
    if Dist_total > Step
        State_new = Node_from + Step_vect;
        Type_steer = 1;
    else
        State_new = Node_to;
        Type_steer = 2;
    end
    
    Judge = IsCollision(State_new, Robot_size, Boundary, Obstacle);
	% ��ײ���
	Num_collision_test = Num_collision_test + 1;
    
    if Judge == 0 %δ������ײ
        Node_new = State_new;
        Cost = Distance(Node_from, Node_new);
    else        %������ײ
        State_collision = State_new;
        Type_steer = 3;
    end

    
end