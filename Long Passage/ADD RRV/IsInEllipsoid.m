%判断点是否在椭球内
function Judge = IsInEllipsoid(Point, Length_axis)
    if size(Point) ~= size(Length_axis)
        disp('Error: Wrong Dimension In IsInellipsoid !!!!');
        return
    end
    
    R = sum((Point .^2) ./ Length_axis); 
    
    if R<=1
        Judge = 1;
    else
        Judge = 0;
    end
    
end 
