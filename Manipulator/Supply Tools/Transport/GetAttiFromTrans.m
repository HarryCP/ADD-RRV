%在二维平面中，从旋转矩阵得到姿态角
function Attitude = GetAttiFromTrans(Trans)
    SA = Trans(2, 1);   %姿态角的cos值
    CA = Trans(1, 1);   %姿态角的sin值
    
    Attitude = asin(SA);
    
    if SA >= 0 && CA <= 0
       Attitude = pi - Attitude; 
    end
    
    if SA <= 0 && CA <= 0
       Attitude = -pi - Attitude; 
    end
    
end