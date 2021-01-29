%用于碰撞检测
function Judge = IsCollision(State_new, Robot_size, Boundary, Obstacle)
    %机器人为圆形
    Robot = zeros(2, 2);
    Robot(:, 1) = State_new;
    Robot(:, 2) = [Robot_size, Robot_size]';

    if State_new(1,1) < Boundary(1, 1)  && State_new(1,1) > Boundary(1, 2) && ... 
            State_new(2,1) <  Boundary(2, 1)  && State_new(2,1) > Boundary(2, 2)
    	Judge = 1;
        return;
    end
    
    %% 此情形下，障碍物分为7个
    Judge = 0;
    if GJK(Robot, 0, Obstacle.NO1, 1) || GJK(Robot, 0, Obstacle.NO2, 1) || GJK(Robot, 0, Obstacle.NO3, 1) || ...
            GJK(Robot, 0, Obstacle.NO4, 1) || GJK(Robot, 0, Obstacle.NO5, 1) || GJK(Robot, 0, Obstacle.NO6, 1) || ...
            GJK(Robot, 0, Obstacle.NO7, 1) 
        
        Judge = 1;
        
    end
    
    
    
    
end