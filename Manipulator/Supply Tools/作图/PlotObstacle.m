%���ڻ����ϰ���
function PlotObstacle(ObstacleData)
    
    %%                 
    %���Ƶ�1���ϰ���         
   
    X1 = ObstacleData.NO1(1, :);
    Y1 = ObstacleData.NO1(2, :);
   
    plot(X1, Y1, 'k', 'LineWidth', 2);
    fill(X1, Y1, 'k');
    
    %%                 
    %���Ƶ�2���ϰ���         
   
    X2 = ObstacleData.NO2(1, :);
    Y2 = ObstacleData.NO2(2, :);
   
    plot(X2, Y2, 'k', 'LineWidth', 2);
    fill(X2, Y2, 'k');
    
    %%                 
    %���Ƶ�3���ϰ���         
   
    X3 = ObstacleData.NO3(1, :);
    Y3 = ObstacleData.NO3(2, :);
   
    plot(X3, Y3, 'k', 'LineWidth', 2);
    fill(X3, Y3, 'k');
    
    
end