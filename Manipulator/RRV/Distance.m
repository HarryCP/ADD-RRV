%用于计算两个状态间距离的度量函数
function Dist = Distance(From, To, Weight, Flag)
    
    %用加权范数将其归一化
    Diff = (From - To);
    if Flag == 2    %表示第三维是姿态
        Diff(3, :) = AttitudeDifference(From(3, :), To(3, :));
    end
    
    Dist =  sum((Weight*Diff) .^ 2) .^ 0.5;
   
end