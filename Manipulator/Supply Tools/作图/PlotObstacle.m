%用于绘制障碍物
function PlotObstacle(ObstacleData)
    
    %%                 
    %绘制第1个障碍物         
   
    X1 = ObstacleData.NO1(1, :);
    Y1 = ObstacleData.NO1(2, :);
   
    plot(X1, Y1, 'k', 'LineWidth', 2);
    fill(X1, Y1, 'k');
    
    %%                 
    %绘制第2个障碍物         
   
    X2 = ObstacleData.NO2(1, :);
    Y2 = ObstacleData.NO2(2, :);
   
    plot(X2, Y2, 'k', 'LineWidth', 2);
    fill(X2, Y2, 'k');
    
    %%                 
    %绘制第3个障碍物         
   
    X3 = ObstacleData.NO3(1, :);
    Y3 = ObstacleData.NO3(2, :);
   
    plot(X3, Y3, 'k', 'LineWidth', 2);
    fill(X3, Y3, 'k');
    
    
end