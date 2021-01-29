%用于计算两个状态间距离的度量函数
function Dist = Distance(From, To, Weight)
    %状态包含二维的位置和一维的姿态
    %用加权范数将其归一化
    Diff = (From - To);
    Diff(3, :) = AttitudeDifference(From(3, :), To(3, :));
    
    Dist =  sum((Weight*Diff) .^ 2) .^ 0.5;
    
end