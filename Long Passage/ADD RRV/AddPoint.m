%����ײ�߽�㼯��������µ�
function [Points_set, Num_points, Flag] = AddPoint(Points_set, Point, Num_points, Weight, Error_goal)
    Flag = 0;
    if Num_points == 0 %��ʾ��ӵĵ�һ����ײ��
        Points_set = Point;
        Num_points = Num_points + 1;
        Flag = 1;
    else
        Dist_temp = Distance(Points_set, Point, Weight);
        if min(Dist_temp) > Error_goal
            Points_set(:, end+1) = Point;
           	%���µĵ㲻�ڼ������棬����Ӵ˵�
            Num_points = Num_points + 1;
            Flag = 1;
        end 
    end
    
    
end