%根据PCA对障碍物中的点的分析结果，对无碰撞的点进行再分析
function [ Points_free_in_ellipsoid, Judge_node_from_in_ellipsoid] = PointsAnalyse(Coeff, Latent, Mu, Points_free, Num_points_free, Dim, Node_from, P_belif)
    % Coeff为协方差矩阵的特性向量矩阵（列向量），变换矩阵
    % 变换后的点：Score = (X-Mu) * Coeff.
    % Latent为各特征向量对应的特征值，从大到小排列
    % Mu为X（原数据）按列的均值
    
    % Points_free_in_ellipsoid 表示在碰撞点包络区域内的无碰撞的点
    % Judge_node_from_in_ellipsoid 判断Node_from是否在碰撞点包络区域内
    Points_free_in_ellipsoid = [];

    Chisquare_value = chi2inv(P_belif, Dim);        	%表示卡方值, P_belif表示置信区间范围
    Length_axis = Chisquare_value * Latent;             %表示超椭球各半轴长的平方；
    
    for kk = 1:Num_points_free
        Point_temp = ((Points_free(:, kk)' - Mu) * Coeff)';
        %先将点变换心新标系下
        
        Judge = IsInEllipsoid(Point_temp, Length_axis);
        %判断点是否在椭球内
        
        if Judge == 1
            Points_free_in_ellipsoid(:, end+1) = Points_free(:, kk);
        end
        
    end
    
    Point_temp = ((Node_from' - Mu) * Coeff)';
    Judge_node_from_in_ellipsoid = IsInEllipsoid(Point_temp, Length_axis);
    %判断Node_from是否在椭球内
    
end
    