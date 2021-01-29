function Point = Support(ShapeA, FlagA, ShapeB, FlagB,  Vector)
    %��ȡVector������Minkowski Difference �ĵ�
    %Flag��ʾ͹������
    PointA = FarthestPoint(ShapeA, FlagA, Vector);
    PointB = FarthestPoint(ShapeB, FlagB, -Vector);
    
    Point = PointA - PointB;

end
