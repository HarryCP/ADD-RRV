%用于绘制障碍物
function PlotObstacle(Obstacle)
    
    %%                 
    %绘制第1个障碍物         
   
    X1 = Obstacle.NO1(1, :);
    Y1 = Obstacle.NO1(2, :);
   
    plot(X1, Y1, 'k', 'LineWidth', 2);
    fill(X1, Y1, 'k');
    
    %%                 
    %绘制第2个障碍物         
   
    X2 = Obstacle.NO2(1, :);
    Y2 = Obstacle.NO2(2, :);
   
    plot(X2, Y2, 'k', 'LineWidth', 2);
    fill(X2, Y2, 'k');
    
    %%                 
    %绘制第3个障碍物         
   
    X3 = Obstacle.NO3(1, :);
    Y3 = Obstacle.NO3(2, :);
   
    plot(X3, Y3, 'k', 'LineWidth', 2);
    fill(X3, Y3, 'k');
    
	%%                 
    %绘制第4个障碍物         
   
    X4 = Obstacle.NO4(1, :);
    Y4 = Obstacle.NO4(2, :);
   
    plot(X4, Y4, 'k', 'LineWidth', 2);
    fill(X4, Y4, 'k');

    %%                 
    %绘制第5个障碍物         
   
    X5 = Obstacle.NO5(1, :);
    Y5 = Obstacle.NO5(2, :);
   
    plot(X5, Y5, 'k', 'LineWidth', 2);
    fill(X5, Y5, 'k'); 
    
    %%                 
    %绘制第6个障碍物         
   
    X6 = Obstacle.NO6(1, :);
    Y6 = Obstacle.NO6(2, :);
   
    plot(X6, Y6, 'k', 'LineWidth', 2);
    fill(X6, Y6, 'k');
    
    %%                 
    %绘制第7个障碍物         
   
    X7 = Obstacle.NO7(1, :);
    Y7 = Obstacle.NO7(2, :);
   
    plot(X7, Y7, 'k', 'LineWidth', 2);
    fill(X7, Y7, 'k');
    
    %%                 
    %绘制第8个障碍物         
   
    X8 = Obstacle.NO8(1, :);
    Y8 = Obstacle.NO8(2, :);
   
    plot(X8, Y8, 'k', 'LineWidth', 2);
    fill(X8, Y8, 'k');
    
    %%                 
    %绘制第9个障碍物         
   
    X9 = Obstacle.NO9(1, :);
    Y9 = Obstacle.NO9(2, :);
   
    plot(X9, Y9, 'k', 'LineWidth', 2);
    fill(X9, Y9, 'k');
end