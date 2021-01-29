%����������
function Solution = TreePlot(Tree, Total_node, State_init, State_goal, Success_num, Success, Robot_size, Obstacle, Weight)
    %���ҵ���Ŀ����Χ�ڵ����еĵ�
    if Success_num == 0
        disp('���棺û���ҵ��յ����Ŀ����Χ��·��');
        %return;

        Node_added = Tree.Node(:,1:Total_node);
        Dist = zeros(3, Total_node);
        Dist(1, :) = Distance(Node_added, State_goal, Weight);    %����ÿ���ڵ㵽Ŀ���ľ���
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
    %������Ϣ

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% �����Ϳռ�켣
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
    %����

    for k = 1:1:Path_iter-1
        Path_temp = [Tree.Node(:, Backtrace_path(k)), Tree.Node(:, Backtrace_path(k+1))]; 
        plot(Path_temp(1, :), Path_temp(2, :), 'r-', 'LineWidth', 2); 
    end
    %��·��
    
    PlotCircle(State_init, 'b-', 1);
    PlotCircle(State_goal, 'r-', 1);
    
%     hold off;
%     
  	box on;
    set(gca,'xtick',[],'xticklabel',[])
    set(gca,'ytick',[],'yticklabel',[])
%     
%     %%  ��ʾ��ͼ
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
            %��ά��ת��Ϊ��ά(��Ȼ��ԭ��)   
            Robot_temp =  Transport_Target(Tree.Node(:,  Backtrace_path(kk))) * Robot_temp;
            
            Robot = Robot_temp(1:Dim, :);
            
            plot(Robot(1, :), Robot(2, :), 'b', 'LineWidth', 2);
            fill(Robot(1, :), Robot(2, :), 'b');
            
            
     
        end
    end
    
%     hold off;
    
    
end