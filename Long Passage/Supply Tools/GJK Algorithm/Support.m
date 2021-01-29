function Point = Support(ShapeA, FlagA, ShapeB, FlagB,  Vector)
    %获取Vector方向上Minkowski Difference 的点
    %Flag表示凸体类型
    PointA = FarthestPoint(ShapeA, FlagA, Vector);
    PointB = FarthestPoint(ShapeB, FlagB, -Vector);
    
    Point = PointA - PointB;

end
