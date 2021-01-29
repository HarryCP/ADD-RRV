%画出搜索树图和运动过程图
function Solution = TreePlot(Tree, Total_node, Success_num, Success, State_goal, End_goal, ObstacleData, Weight )

    %先找到在目标误差范围内的所有的点
    if Success_num == 0
        disp('警告：没有找到终点符合目标误差范围的路径');
        %return;

        Node_added = Tree.End(:,1:Total_node);
        Dist = zeros(3, Total_node);
        Dist(1, :) = Distance(Node_added, End_goal, Weight, 2);    	%计算每个节点到目标点的距离
        Dist(2, :) = 1 : Total_node;                                %每个节点的编号       
        Dist = sortrows(Dist')' ;                                   %按距离从小到大排序
        
        Dist(2:3, :) = Dist(1:2,:);
        Dist(1, :) = FindTotalCost(Tree, Dist(3, :));            	%找到每个节点对应的代价函数值
        %Distance，第一行是代价，第二行是距离，第三行是索引
        
        Nearest_goal_index = Dist(3,1);
        disp(['距离终点误差为 ' num2str(Dist(2,1))]);
        
    else
        Dist = zeros(3, Success_num);
        Dist(1:2, :) = Success(:, 1:Success_num);             %第一行为索引，第二行为距离
    
        %开始找代价函数值最低的路径
        Dist(3, :) = Dist(1, :);                           	%将第一行的值赋给第三行
        Dist(1, :) = FindTotalCost(Tree, Dist(3, :));     	%找到每个节点对应的代价函数值
        Dist = sortrows(Dist')';                            %按代价函数值进行从小到大排序
        Nearest_goal_index = Dist(3,1);                     %搜索到的离目标点最优的点的编号
        %Distance，第一行是代价，第二行是距离，第三行是索引
    
        disp('成功：找到终点符合目标误差范围的路径！！！');
        disp(['距离终点误差为 ' num2str(Dist(2,1))]);
    end
    
    
    %回溯路径
    Current_index = Nearest_goal_index;                     %当前回溯的路径点编号
    Path_iter = 1;                                          %路径点个数
    Backtrace_path = zeros(1,1);                            %回溯路径(以编号形式储存)
    while Current_index ~= 1
       Backtrace_path(Path_iter) =  Current_index;
       Path_iter = Path_iter + 1;
       Current_index = Tree.Parent(Current_index);
    end
    
    Backtrace_path(Path_iter) =  Current_index;
    %找到路径
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%
    Solution = struct;
    Solution.Distance = Dist(2, :);
    Solution.Cost = Dist(1, :);
    Solution.FinalIndex = Dist(3, :);
    
    Solution.OptimalDistance = Dist(2, 1);
    Solution.OptimalCost = Dist(1, 1);
    Solution.OptimalFinalIndex = Dist(3, 1);
    Solution.BacktracePath = Backtrace_path;
    %保存信息
    %%
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %画末端轨迹             
    figure
	set(gcf(), 'Renderer', 'opengl');
	axis([-2 12 -2 12]);
	axis square;
	hold on;
    
    PlotObstacle(ObstacleData);
	
    for k = Total_node:-1:2
        Path_temp = [Tree.End(:, k),Tree.End(:, Tree.Parent(k))];        
        plot(Path_temp(1, :), Path_temp(2, :), 'g-', 'LineWidth', 0.5); 
    end
    %画树
    
    for k = 1:1:Path_iter-1
        Path_temp = [Tree.End(:, Backtrace_path(k)), Tree.End(:, Backtrace_path(k+1))]; 
        plot(Path_temp(1, :), Path_temp(2, :), 'r-', 'LineWidth', 2); 
    end
    
    box on;
    set(gca,'xtick',[],'xticklabel',[])
    set(gca,'ytick',[],'yticklabel',[])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
end