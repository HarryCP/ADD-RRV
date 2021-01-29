%画出搜索树
function Solution = TreePlot(Tree, Total_node, State_init, State_goal, Success_num, Success, Robot_size, Obstacle, Weight)
    %先找到在目标误差范围内的所有的点
    if Success_num == 0
        disp('警告：没有找到终点符合目标误差范围的路径');
        %return;

        Node_added = Tree.Node(:,1:Total_node);
        Dist = zeros(3, Total_node);
        Dist(1, :) = Distance(Node_added, State_goal, Weight);    %计算每个节点到目标点的距离
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
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 画构型空间轨迹
    figure
	set(gcf(), 'Renderer', 'opengl');
	axis([0 12 0 8.5]);
	axis square;
	hold on;
    
    PlotObstacle(Obstacle);
    
    for k = Total_node:-1:2
        Path_temp = [Tree.Node(:, k),Tree.Node(:, Tree.Parent(k))];        
        plot(Path_temp(1, :), Path_temp(2, :), 'g-', 'LineWidth', 0.5); 
    end
    %画树

    for k = 1:1:Path_iter-1
        Path_temp = [Tree.Node(:, Backtrace_path(k)), Tree.Node(:, Backtrace_path(k+1))]; 
        plot(Path_temp(1, :), Path_temp(2, :), 'r-', 'LineWidth', 2); 
    end
    %画路径
    
    PlotCircle(State_init, 'b-', 1);
    PlotCircle(State_goal, 'r-', 1);
    
%     hold off;
%     
  	box on;
    set(gca,'xtick',[],'xticklabel',[])
    set(gca,'ytick',[],'yticklabel',[])
%     
%     %%  画示意图
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	figure 
	set(gcf(), 'Renderer', 'opengl');
	axis([0 12 0 8.5]);
	axis square;
  	hold on;
    
	PlotObstacle(Obstacle);
        
	box on;
	set(gca,'xtick',[],'xticklabel',[])
	set(gca,'ytick',[],'yticklabel',[])
    

    for kk = Path_iter:-1: 1
        if(mod(kk, 5) == 0) || kk == 1 || kk == Path_iter
            
            [Dim, Num] = size(Robot_size);
            Robot_temp = zeros(4, Num);
            Robot_temp(1:Dim, :) = Robot_size;
            Robot_temp(4, :) = 1;
            %将维度转化为四维(任然在原点)   
            Robot_temp =  Transport_Target(Tree.Node(:,  Backtrace_path(kk))) * Robot_temp;
            
            Robot = Robot_temp(1:Dim, :);
            
            plot(Robot(1, :), Robot(2, :), 'b', 'LineWidth', 2);
            fill(Robot(1, :), Robot(2, :), 'b');
            
            
     
        end
    end
    
%     hold off;
    
    
end