%用于碰撞检测
function Judge = IsCollision(State_robot, Robot_size, Boundary, Obstacle)
	Judge = 0;
        
    [Dim, Num] = size(Robot_size);
    Robot_temp = zeros(4, Num);
    Robot_temp(1:Dim, :) = Robot_size;
    Robot_temp(4, :) = 1;
    %将维度转化为四维(任然在原点)
    
    
    Robot_temp =  Transport_Target(State_robot) * Robot_temp;
    
    Robot = Robot_temp(1:Dim, :);
    
    %% 判断状态是否会越界
    if State_robot(1,:) < Boundary(1, 1) || State_robot(1,:) > Boundary(1, 2) || ...
            State_robot(2,:) < Boundary(2, 1) || State_robot(2,:) > Boundary(2, 2)
        Judge = 1;
        return
    end
        
        
    
    %% 此情形下，障碍物分为7个

    if GJK(Robot, 1, Obstacle.NO1, 1) || GJK(Robot, 1, Obstacle.NO2, 1) || GJK(Robot, 1, Obstacle.NO3, 1) ...
            || GJK(Robot, 1, Obstacle.NO4, 1) || GJK(Robot, 1, Obstacle.NO5, 1) || GJK(Robot, 1, Obstacle.NO6, 1) ...
            || GJK(Robot, 1, Obstacle.NO7, 1) || GJK(Robot, 1, Obstacle.NO8, 1) || GJK(Robot, 1, Obstacle.NO9, 1) 
        
        Judge = 1;
        
    end
    
    
    
    
end