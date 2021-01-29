%向碰撞边界点集合中添加新点
function [Points_set, Num_points, Flag] = AddPoint(Points_set, Point, Num_points, Error_goal)
    Flag = 0;
    if Num_points == 0 %表示添加的第一个碰撞点
        Points_set = Point;
        Num_points = Num_points + 1;
        Flag = 1;
    else
        Dist_temp = Distance(Points_set, Point);
        if min(Dist_temp) > Error_goal
            Points_set(:, end+1) = Point;
           	%若新的碰撞点不在碰撞边界集里面没有，则添加此点
            Num_points = Num_points + 1;
            Flag = 1;
        end 
    end
    
end