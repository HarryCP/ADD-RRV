%����ײ�߽�㼯��������µ�
function [Points_set, Num_points, Flag] = AddPoint(Points_set, Point, Num_points, Error_goal)
    Flag = 0;
    if Num_points == 0 %��ʾ��ӵĵ�һ����ײ��
        Points_set = Point;
        Num_points = Num_points + 1;
        Flag = 1;
    else
        Dist_temp = Distance(Points_set, Point);
        if min(Dist_temp) > Error_goal
            Points_set(:, end+1) = Point;
           	%���µ���ײ�㲻����ײ�߽缯����û�У�����Ӵ˵�
            Num_points = Num_points + 1;
            Flag = 1;
        end 
    end
    
end