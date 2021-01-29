function [Judge , NewSimplix, NewDirection] = IsContainOrigin(OldSimplix, OldDirection, Flag)
    %判断原点是否在单纯形里面
	[m, n] = size(OldSimplix);
    
    NewSimplix = OldSimplix;
    NewDirection = zeros(m, 1);
    
    %m表示空间维度，n表示单纯形顶点数目
    %目前只考虑了二维的情况
    PointA = OldSimplix(:, 1);
    PointB = OldSimplix(:, 2);
    PointC = OldSimplix(:, 3);
    Origin = zeros(m, 1);
    
    VectorAO = Origin - PointA;
    VectorAB = PointB - PointA;
    VectorAC = PointC - PointA;
    
    if Flag > 2    %单纯形是三角形
        PerpAB = DoubleCross(VectorAC, VectorAB, VectorAB);     %AB的垂线，并指向单纯形外部
        PerpAC = DoubleCross(VectorAB, VectorAC, VectorAC);     %AC的垂线，并指向单纯形外部
        
        if PerpAB' * VectorAO > 0   %原点在单纯形AB边的外部
            NewSimplix(:, 2) = OldSimplix(:, 1);
            NewSimplix(:, 3) = OldSimplix(:, 2);
            NewDirection = PerpAB;
            Judge = 0;
        else
            if PerpAC' * VectorAO > 0   %原点在单纯形AC边的外部
                NewSimplix(:, 2) = OldSimplix(:, 1);
                NewSimplix(:, 3) = OldSimplix(:, 3);
                NewDirection = PerpAC;
                Judge = 0;
            else
                Judge = 1;           
            end 
        end
        
    else        %单纯形是一条直线
        NewDirection = DoubleCross(VectorAB, VectorAO, VectorAB);
        NewSimplix(:, 2) = OldSimplix(:, 1);
       	NewSimplix(:, 3) = OldSimplix(:, 2);
        Judge = 0;
        
    end
    
    


end