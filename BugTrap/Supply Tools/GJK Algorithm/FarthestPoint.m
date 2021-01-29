function Point = FarthestPoint(Shape, Flag, Vector)
	%在形状Shape内得到Vector方向上最远的顶点
	%若Falg=0，表示形状为圆；Shape第一列表示圆心坐标，第二列表示半径（任意一个元素都是半径的值）
	%若Flag=1，表示形状为凸多边形；Shape每一列表示一个顶点坐标
    if Flag == 1
        [~, n] = max(Vector' * Shape);
        Point = Shape(:, n);
    else
        if Flag == 0
            Point = Shape(:, 1) + Shape(1, 2)*Vector/norm(Vector);
        else
           fprint('ERROR: Wrong Shape In FarthestPoint of GJK !!!'); 
        end
    end
        
end
