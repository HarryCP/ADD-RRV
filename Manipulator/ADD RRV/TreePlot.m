%����������ͼ���˶�����ͼ
function Solution = TreePlot(Tree, Total_node, Success_num, Success, State_goal, End_goal, ObstacleData, Weight )

    %���ҵ���Ŀ����Χ�ڵ����еĵ�
    if Success_num == 0
        disp('���棺û���ҵ��յ����Ŀ����Χ��·��');
        %return;

        Node_added = Tree.End(:,1:Total_node);
        Dist = zeros(3, Total_node);
        Dist(1, :) = Distance(Node_added, End_goal, Weight, 2);    	%����ÿ���ڵ㵽Ŀ���ľ���
        Dist(2, :) = 1 : Total_node;                                %ÿ���ڵ�ı��       
        Dist = sortrows(Dist')' ;                                   %�������С��������
        
        Dist(2:3, :) = Dist(1:2,:);
        Dist(1, :) = FindTotalCost(Tree, Dist(3, :));            	%�ҵ�ÿ���ڵ��Ӧ�Ĵ��ۺ���ֵ
        %Distance����һ���Ǵ��ۣ��ڶ����Ǿ��룬������������
        
        Nearest_goal_index = Dist(3,1);
        disp(['�����յ����Ϊ ' num2str(Dist(2,1))]);
        
    else
        Dist = zeros(3, Success_num);
        Dist(1:2, :) = Success(:, 1:Success_num);             %��һ��Ϊ�������ڶ���Ϊ����
    
        %��ʼ�Ҵ��ۺ���ֵ��͵�·��
        Dist(3, :) = Dist(1, :);                           	%����һ�е�ֵ����������
        Dist(1, :) = FindTotalCost(Tree, Dist(3, :));     	%�ҵ�ÿ���ڵ��Ӧ�Ĵ��ۺ���ֵ
        Dist = sortrows(Dist')';                            %�����ۺ���ֵ���д�С��������
        Nearest_goal_index = Dist(3,1);                     %����������Ŀ������ŵĵ�ı��
        %Distance����һ���Ǵ��ۣ��ڶ����Ǿ��룬������������
    
        disp('�ɹ����ҵ��յ����Ŀ����Χ��·��������');
        disp(['�����յ����Ϊ ' num2str(Dist(2,1))]);
    end
    
    
    %����·��
    Current_index = Nearest_goal_index;                     %��ǰ���ݵ�·������
    Path_iter = 1;                                          %·�������
    Backtrace_path = zeros(1,1);                            %����·��(�Ա����ʽ����)
    while Current_index ~= 1
       Backtrace_path(Path_iter) =  Current_index;
       Path_iter = Path_iter + 1;
       Current_index = Tree.Parent(Current_index);
    end
    
    Backtrace_path(Path_iter) =  Current_index;
    %�ҵ�·��
    
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
    %������Ϣ
    %%
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %��ĩ�˹켣             
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
    %����
    
    for k = 1:1:Path_iter-1
        Path_temp = [Tree.End(:, Backtrace_path(k)), Tree.End(:, Backtrace_path(k+1))]; 
        plot(Path_temp(1, :), Path_temp(2, :), 'r-', 'LineWidth', 2); 
    end
    
    box on;
    set(gca,'xtick',[],'xticklabel',[])
    set(gca,'ytick',[],'yticklabel',[])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
end