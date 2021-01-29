%根据中心点对点集姿态进行统一化
function Points_set = UnifyAttitude(Points_set, Node_from)
    [~, Num_points] = size(Points_set);
    
    for kk = 1:Num_points
        if AttitudeDifference(Node_from(3, :), Points_set(3, kk)) < abs(Node_from(3, :) - Points_set(3, kk))
            if Node_from(3, :) < 0
                Points_set(3, kk) = Points_set(3, kk) - 2*pi;
            else
                Points_set(3, kk) = Points_set(3, kk) + 2*pi;
            end
        end
    end
end