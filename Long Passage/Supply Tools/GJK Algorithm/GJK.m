function Judge = GJK(ShapeA, FlagA, ShapeB, FlagB)
%对给定的两个凸体ShapeA、ShapeB进行碰撞检测
%目前仅考虑了2维的情况
%若Falg=0，表示形状为圆；Shape第一列表示圆心坐标，第二列表示半径（任意一个元素都是半径的值）
%若Flag=1，表示形状为凸多边形；Shape每一列表示一个顶点坐标
	
Judge = 0;
    
    [m1, n1] = size(ShapeA);
    [m2, n2] = size(ShapeB);
    % m表示形状的维度，n表示多边形（多面体）的顶点数 
    
    if m1 ~= m2
        disp('ERROR: Wrong Dimension In GJK !!!');
        return;
    end
    %保证进行碰撞检测的两个图形维度相同
    
    Direction = rand(m1, 1);
    %寻找一个随机方向
    Simplix = repmat(Support(ShapeA, FlagA, ShapeB, FlagB, Direction), 1, m1+1); 
    %得到单纯形中的第一个点
    Direction = -Direction;
    %将方向向量反向，为了得到下一个点
    k = 1;
    %表示检测的点的数量
    
    while true
        k = k + 1;
        Simplix(:, 1) = Support(ShapeA, FlagA, ShapeB, FlagB, Direction);
        %向单纯形中添加一个新点
        if Simplix(:,1)'*Direction < 0
            Judge = 0;
            return;
            %如果新添加的点从Vector方向上没有穿过原点，
            %那么不可能发生碰撞
            %(因为新添加的点在Minkowski Difference的边界上)
        else
            [Judge , Simplix, Direction] = IsContainOrigin(Simplix, Direction, k);
            %判断单纯形是否包含原点
            %若单纯形包含原点，则发生碰撞
            %否则，还不能确定是否发生碰撞，
           	%寻找离原点最近的那条边，并且以这条边指向原点的法向量方向作为搜索方向
            if Judge == 1
                return;
                %若单纯形包含原点，则发生碰撞
            end
           
        end
         
    end
   
end
