%对随机采样的点继续进行分类，分为发生碰撞的和无碰撞的
function [Points_obstacle, Points_free] = ClassfyPoints(Points_set, Robot_size, Boundary, Obstacle)
    [Dim, Num] = size(Points_set);
    %获取点集的维度和数量
    
    Points_obstacle = [];
    Points_free = [];
    
    Flag = zeros(1, Num);
    % 用来表明当前点是否发生碰撞
    % 0 表示未碰撞
    % 1 表示碰撞
    
    for kk = 1:Num
        Judge = IsCollision(Points_set(:, kk), Robot_size, Boundary, Obstacle);
        %对每个点进行碰撞检测
        
        if Judge ~= 0 
            Flag(kk) = 1;
        end
        %对点进行分类
        
    end
       
    Points_obstacle = Points_set(:, Flag == 1);   
    Points_free = Points_set(:, Flag == 0);
    
end

