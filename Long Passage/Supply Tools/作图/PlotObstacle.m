%���ڻ����ϰ���
function PlotObstacle(Obstacle)
    
    %%                 
    %���Ƶ�1���ϰ���         
   
    X1 = Obstacle.NO1(1, :);
    Y1 = Obstacle.NO1(2, :);
   
    plot(X1, Y1, 'k', 'LineWidth', 2);
    fill(X1, Y1, 'k');
    
    %%                 
    %���Ƶ�2���ϰ���         
   
    X2 = Obstacle.NO2(1, :);
    Y2 = Obstacle.NO2(2, :);
   
    plot(X2, Y2, 'k', 'LineWidth', 2);
    fill(X2, Y2, 'k');
    
    %%                 
    %���Ƶ�3���ϰ���         
   
    X3 = Obstacle.NO3(1, :);
    Y3 = Obstacle.NO3(2, :);
   
    plot(X3, Y3, 'k', 'LineWidth', 2);
    fill(X3, Y3, 'k');
    
	%%                 
    %���Ƶ�4���ϰ���         
   
    X4 = Obstacle.NO4(1, :);
    Y4 = Obstacle.NO4(2, :);
   
    plot(X4, Y4, 'k', 'LineWidth', 2);
    fill(X4, Y4, 'k');

    %%                 
    %���Ƶ�5���ϰ���         
   
    X5 = Obstacle.NO5(1, :);
    Y5 = Obstacle.NO5(2, :);
   
    plot(X5, Y5, 'k', 'LineWidth', 2);
    fill(X5, Y5, 'k'); 
    
    %%                 
    %���Ƶ�6���ϰ���         
   
    X6 = Obstacle.NO6(1, :);
    Y6 = Obstacle.NO6(2, :);
   
    plot(X6, Y6, 'k', 'LineWidth', 2);
    fill(X6, Y6, 'k');
    
    %%                 
    %���Ƶ�7���ϰ���         
   
    X7 = Obstacle.NO7(1, :);
    Y7 = Obstacle.NO7(2, :);
   
    plot(X7, Y7, 'k', 'LineWidth', 2);
    fill(X7, Y7, 'k');
    
    %%                 
    %���Ƶ�8���ϰ���         
   
    X8 = Obstacle.NO8(1, :);
    Y8 = Obstacle.NO8(2, :);
   
    plot(X8, Y8, 'k', 'LineWidth', 2);
    fill(X8, Y8, 'k');
    
    %%                 
    %���Ƶ�9���ϰ���         
   
    X9 = Obstacle.NO9(1, :);
    Y9 = Obstacle.NO9(2, :);
   
    plot(X9, Y9, 'k', 'LineWidth', 2);
    fill(X9, Y9, 'k');
end