%��������״̬
function [Node_new, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                Steer(Tree, From_index, Node_to, Step, Robot_size, Boundary, Obstacle, Num_collision_test, Weight)  
	%Type_steer = 1, ǰ��һ��
    %Type_steer = 2�����������
    %Type_steer = 3, ������ײ
    
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
    
    Dist_total = Distance(Node_from, Node_to, Weight);      % �������ܳ���
    Vect = Weight*(Node_to - Node_from);                    % ��չ���� 
    Step_vect = Vect*(Step/Dist_total);                     % ��չ�����ϵĲ���
       
    if Dist_total > Step
        State_new = Node_from + Step_vect;
        State_new(3, :) = GetAttiFromTrans(RotZ(State_new(3, :)));  %��ĩ����̬����У��
        Type_steer = 1;
        %ǰ��һ��
    else
        State_new = Node_to;
        State_new(3, :) = GetAttiFromTrans(RotZ(State_new(3, :)));  %��ĩ����̬����У��
        Type_steer = 2;
        %����Ŀ�������
    end
    
    Judge = IsCollision(State_new, Robot_size, Boundary, Obstacle);
	% ��ײ���
	Num_collision_test = Num_collision_test + 1;    
           
    if Judge == 0 %δ������ײ
        Node_new = State_new;
        Cost = Distance(Node_from, Node_new, Weight);
    else        %������ײ
        State_collision = State_new;
        Type_steer = 3;
    end
    
    
    
    
    
end