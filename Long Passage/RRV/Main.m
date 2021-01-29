% RRV
% Copyrigt by Peng Cai 
% 01/29/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ÿ����չ������ײ�߽���Ϣ��¼������
%����ڱ߽繹�͸���������������ӵ���ײ������С���½磻
%����������û�б���ȫ̽����
%������û�����RRV������չ��
%��Ҫ��������RRV��Ѱ��խͨ����ں���խͨ��������ƣ�
%���ǵ�����RRV��Ҫ���д����������������ײ��⣬��������
%��˵��ڱ߽繹�͸����Ѽ����㹻����ײ����Ϣʱ��
%������ײ��ָ����չ��ͬʱ�ܹ���ֹ��չ�����͹�ϰ�����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Result = Main()
% ��ͼΪ�����
% clear
% clc

addpath('~\Long Passage\Supply Tools\Supply Tools\��ͼ');
addpath('~\Long Passage\Supply Tools\Supply Tools\GJK Algorithm');
addpath('~\Long Passage\Supply Tools\Supply Tools\Transport');
%����ʵ��·���޸�

%% ��ʼ�����Ͳ�������
Boundary = [   0,  12;
               0, 8.5; 
             -pi,  pi];
% %��ͼ�߽�

Node_init = [1, 5, pi/2]';   %��ʼλ��
Node_goal = [11, 5, pi/2]';   %Ŀ��λ��

delta = 0.7;

Robot_size = ShapeToPoint([0, 0]', delta*0.5, delta*0.1);   %��������ԭ��ʱ��״̬

Weight = diag([1, 1, 0.5]); %diag([1, 1, 1])
Weight_1 = eye(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Obstacle = struct;  %�ϰ������
Obstacle.NO1 = [2, 2,   3, 4, 4, 2 
                0, 3, 4.7, 3, 0, 0];

Obstacle.NO2 = [4, 4,   5, 6, 6, 4 
                0, 3, 4.7, 3, 0, 0];
            
Obstacle.NO3 = [6, 6,   7, 8, 8, 6 
                0, 3, 4.7, 3, 0, 0];            
            
Obstacle.NO4 = [8, 8,   9, 10, 10, 8 
                0, 3, 4.7,  3,  0, 0];            
            
Obstacle.NO5 = [  2,   2,   3,   3,   2
                8.5, 3.8, 5.5, 8.5, 8.5]; 

Obstacle.NO6 = [   3,   3,   4,   5,   5,   3
                 8.5, 5.5, 3.8, 5.5, 8.5, 8.5]; 

Obstacle.NO7 = [   5,   5,   6,   7,   7,   5
                 8.5, 5.5, 3.8, 5.5, 8.5, 8.5]; 

Obstacle.NO8= [   7,   7,   8,   9,   9,   7
                8.5, 5.5, 3.8, 5.5, 8.5, 8.5]; 

Obstacle.NO9 = [   9,   9,  10,  10,   9
                 8.5, 5.5, 3.8, 8.5, 8.5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��������
Max_iter = 1*10^5;                      %����������
Step = 0.1;                             %������ÿһ������
Error_goal = 0.01;                      %�������

P_sample = 0.3;                         %��Ŀ���Ϊ������ĸ���
P_belief = 0.60;                       	%�������䷶Χ����[0, 1]��ѡȡ
P_bridge = 0.8;                         %����PCAƫ���Ų�ĸ��� 

Radius = 5*Step;                        %�����뾶

Num_obstacle_limit = 250;                      	%�����ж���ڴ�����ײ���Ƿ��㹻�ḻ,300
Num_max_bridge_test = Num_obstacle_limit;       %����Ų��Դ���
Num_near_sample = 300;                          %�ڽӴ����͸����������������������

Num_collision_test = 0;                         %��ײ������
Num_collision_points = 0;                       %��������ײ��
Num_entrance_points = 0;                        %��⵽����ڵ�
Num_narrow_points = 0;                          %��ʾ��խͨ���еĵ�
Num_PCA_compute_1 = 0;                          %RRV������PCA�����Ĵ���
Num_PCA_compute_2 = 0;                          %���ñ߽�����PCA�����Ĵ���

%% ����һ����
Tree = struct;
Num_node = 1*10^4;
Dim = 3;
Tree.Node = zeros(Dim, Num_node );                                        	%��ʾ�ڵ�
Tree.Cost = zeros(1, Num_node);                                             %��ʾ����һ���ڵ㵽��ǰ�ڵ�Ĵ���
Tree.Parent = zeros(1, Num_node, 'int32');                                  %��¼�ڵ�ĸ��ڵ�����
Tree.Children = zeros(Num_node, Num_node, 'int32');                         %��¼ÿ���ڵ��µ��ӽڵ�����
Tree.Children_num = zeros(1, Num_node, 'int32');                            %��¼ÿ���ڵ��µ��ӽڵ�����
Tree.Type = zeros(1, Num_node);                                             %��¼ÿ���������:0,��ͨ���ͣ�1��ڹ��ͣ�2�ر߽繹�ͣ�3խͨ���ڹ��ͣ�4ͨ���Ų��ԵĹ���

Tree.Node(:, 1) = Node_init;

Node_added_index = 2;                               %��ǰ��ӵĵ�ı��

Collision_boundary_set = [];            %���ڼ�¼������ײ�߽��
Entrance_set = [];                      %���ڼ�¼��⵽����ڵ�
Entrance_flag = [];                     %���ڼ�¼�ҵ�������Ƿ��Ѿ������ʣ�0��ʾδ���ʣ�1��ʾ����
Narrow_set = [];                        %��ʾ��խͨ���еĵ�

Success_num = 0;                        %��ʾ�ɹ��ҵ�·���Ĵ���
Success = zeros(2, Num_node);           %��ʾ���ҵ���·�������һ���ڵ�������;���

%% ���������ڲ�������������
%���õ�λ���ڵĲ�������
Seed_num = 500000;   %���Ӹ���
Seed_unit_ball = SampleUnitBall(Dim, Seed_num);

%% RRT��������
rng('shuffle');

tic;
% ��ʼ��ʱ

for Iter_num = 1:Max_iter

%     if(mod(Iter_num, 100) == 0)
%         disp([num2str(Iter_num) ' �ε��������� ' num2str(Node_added_index-1) ' ���ڵ㣬���� ' num2str(toc) ' s']);
%     end
    
    Node_rand = RandomSample( P_sample, Boundary, Node_goal );
    %�������    
    
    [Node_nearest_index, Dist_min] = Nearest(Tree, Node_rand, Node_added_index, Weight);
    %Ѱ������Ľڵ�
    
    From_index = Node_nearest_index;
    
	Num_extend = 1; 
    Num_extand_max = 1; %��ʾ�����չ����:1/+inf
    
    while true
        [Node, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                Steer(Tree, From_index, Node_rand, Step, Robot_size, Boundary, Obstacle, Num_collision_test, Weight);
     	%Type_steer = 1, ǰ��һ��
        %Type_steer = 2�����������
        %Type_steer = 3, ������ײ
        
        if Judge == 0
            Tree.Node(:, Node_added_index) = Node;
            Tree.Cost(:, Node_added_index) = Cost;
            Tree.Parent(Node_added_index) = From_index;
            Tree.Children_num(From_index) = Tree.Children_num(From_index) + 1;
            Tree.Children(Tree.Children_num(From_index), From_index) = Node_added_index;  
            Tree.Type(:, Node_added_index) = 0;
        
            Tree.Children_num(Node_added_index) = 0;
            Tree.Children(1, Node_added_index) = 0; %��������ӽڵ������ʼ��
        
            Node_added_index = Node_added_index + 1;
        
            Dist_temp = Distance(Node, Node_goal, Weight);
        
            if Dist_temp < Error_goal
                Success_num = Success_num + 1;
                Success(1, Success_num) = Node_added_index - 1;
                Success(2, Success_num) = Dist_temp;
            
                disp(['��' num2str(Success_num) '�γɹ��ҵ���' num2str(Success_num) '·������ ' num2str(toc) ' s']);
                break;
            end
        
        else
            [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight, Error_goal);
            break;
        end
        
        if  Num_extend >= Num_extand_max || Type_steer == 2    %����������չ����
            break
        else
            From_index = Node_added_index - 1;
            Num_extend = Num_extend + 1;
        end
        
    end
    
    
    if Success_num >= 1
        break
    end
    
%     continue
    
    %% ���濪ʼ���û�ȡ�Ĺ��Ϳռ���Ϣ������չ
    if Type_steer ~= 3  %�����������ײ�������в��費����
        continue
    end
    
    IsContinue = true;
    Num_continue = 0;
    while IsContinue
    
        Type_point = 0;
        %��ʾ�ؼ����͵����ͣ�
        %0 ��ʾ��ʼ����
        %1 ��ʾPCA���������ű߽���չ
        %2 ��ʾPCA�������ҵ��Ŀ������
        %3 ��ʾPCA�������ҵ���խͨ��

        Flag_new_entrance = 0;  %��ʾ�Ƿ��ҵ��µ���ڵ㣺0��ʾ��1��ʾ��

        Node_from = Tree.Node(:, From_index);
        Points_obstacle = Near(Collision_boundary_set, Num_collision_points, Node_from, Radius, Weight);
        %�Ѽ���������ײ����   
        [~, Num_points_obstacle] = size(Points_obstacle);

        Num_node_to = 0;    
        if Num_points_obstacle <= Num_obstacle_limit
            Points_sample = SampleAround(Node_from, Num_near_sample, Radius, Boundary, Seed_num, Seed_unit_ball, Weight);
            %��ĳ�㸽�������������

            [Points_obstacle, Points_free] = ClassfyPoints(Points_sample, Robot_size, Boundary, Obstacle);
            %����������ĵ�������з��࣬��Ϊ������ײ�ĺ�����ײ��

            [~, Num_points_free] = size(Points_free);

            if length(Points_obstacle) <= Dim+1
                break;
            end
            %�����ȡ����ײ����٣��������һ����

            [Coeff_obs, Score_obs, Latent_obs, Tsquared_obs, Explained_obs, Mu_obs] = pca((Weight*Points_obstacle)');
            % ���ϰ����еĵ����PCA����,����
            % CoeffΪЭ�������������������������������任����
            % ScoreΪ�任��ĵ�,Score = (X-Mu) * Coeff.
            % LatentΪ������������Ӧ������ֵ���Ӵ�С����
            % Explainedÿһ�����ɷֵĹ��ױ�
            % MuΪX��ԭ���ݣ����еľ�ֵ

            Num_PCA_compute_1 = Num_PCA_compute_1 + 1;
            Num_PCA_compute_2 = Num_PCA_compute_2 + 1;       

            [Points_free_in_ellipsoid, Judge_node_from_in_ellipsoid] = PointsAnalyse(Coeff_obs, Latent_obs, Mu_obs, Points_free, Num_points_free, Dim, Node_from, P_belief, Weight);
            %����PCA���ϰ����еĵ�ķ��������������ײ�ĵ�����ٷ���

            Node_rand = UnifyAttitude(Node_rand, Node_from);
            %�������ĵ����̬����ͳһ�� ����Node_fromͳһ��

            if isempty(Points_free_in_ellipsoid)
                %�ϰ������������ڲ���������ײ��
                %Node_from��͹�ϰ���߽�
                Num_node_to = 1;
                Node_to = NodeProject(Node_from, Node_rand, Coeff_obs, Latent_obs);            
                Type_point = 1;  %��ʾPCA���������ű߽���չ
            else
                if Judge_node_from_in_ellipsoid == 0
                    % Node_from��խͨ����ڴ�
                    Num_node_to = 1;
                    Node_to = NodeProject(Node_from, Node_rand, Coeff_obs, Latent_obs);
                    Type_point = 2;  %��ʾPCA�������ҵ����

                    %%%%%%%�����ڽڵ�%%%%%%%%%
                    [~, Num_temp] = size(Points_free_in_ellipsoid);
                    Points_center = sum(Points_free_in_ellipsoid, 2)/Num_temp;
                    [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Points_center, Num_entrance_points, Weight, Error_goal);                
                    if IsSucceedAdd == 1
                        Entrance_flag(:, Num_entrance_points) = 0;               
                        %��¼���ܵ�ͨ�����                   
                        Flag_new_entrance = 1;  %��ʾ�ҵ����                      
                    end                
                else
                    [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight*Points_free)');
                    Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
                    Num_node_to = 1;
                    Node_temp_goal = 100*(Node_rand - Node_from) + Node_from;    %��Ŀ���ӳ���ʹ������ͨ���о�������չ
                    Node_to = NodeProject(Node_from, Node_temp_goal, Coeff_free, Latent_free);
                    Type_point = 3;  %��ʾPCA��������ͨ����
                end
            end

        end

        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%���������������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%�ȼ�¼��ʼ�ڵ㣬���ڶ����չ
        NODE_FROM = Node_from;
        FROM_INDEX = From_index;

        %%%%���1������ҵ�����ڴ��Ľڵ㣬�ȶ���ڽڵ������չ
        if Flag_new_entrance == 1
            Node_from = NODE_FROM;
            From_index = FROM_INDEX;

            Num_extend = 1;
            Num_extand_max = +inf;  %��ʾ�����չ����
            while true
                [Node, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                        Steer(Tree, From_index, Entrance_set(:, Num_entrance_points), Step, Robot_size, Boundary, Obstacle, Num_collision_test, Weight);
                %Type_steer = 1, ǰ��һ��
                %Type_steer = 2�����������
                %Type_steer = 3, ������ײ

                if Judge == 0
                    Tree.Node(:, Node_added_index) = Node;
                    Tree.Cost(:, Node_added_index) = Cost;
                    Tree.Parent(Node_added_index) = From_index;
                    Tree.Children_num(From_index) = Tree.Children_num(From_index) + 1;
                    Tree.Children(Tree.Children_num(From_index), From_index) = Node_added_index;  
                    Tree.Type(:, Node_added_index) = 0;

                    Tree.Children_num(Node_added_index) = 0;
                    Tree.Children(1, Node_added_index) = 0; %��������ӽڵ������ʼ��

                    Node_added_index = Node_added_index + 1;

                    Dist_temp = Distance(Node, Node_goal, Weight);

                    if Dist_temp < Error_goal
                        Success_num = Success_num + 1;
                        Success(1, Success_num) = Node_added_index - 1;
                        Success(2, Success_num) = Dist_temp;

                        disp(['��' num2str(Success_num) '�γɹ��ҵ���' num2str(Success_num) '·������ ' num2str(toc) ' s']);
                        break;
                    end

                else
                    [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight, Error_goal);

                    %�����ײ���;���Ŀ����ڹ��ͣ���Ŀ����ڹ��ͱ��Ϊһ̽��״̬
                    if Distance(Entrance_set(:, Num_entrance_points), State_collision, Weight) < Error_goal
                        Entrance_flag(:, Num_entrance_points) = 1;
                    end

                    break;
                end

                if  Num_extend >= Num_extand_max || Type_steer == 2    
                    %����������չ����, ֱ��������ڲŻ�ֹͣ
                    Entrance_flag(:, Num_entrance_points) = 1;
                    %��ʾ��ڼ����У��˵��Ѿ����ҵ�

                    Tree.Type(:, Node_added_index-1) = 1; 
                    %��ʾ���У�����ӵĵ�Ϊ��ڵ�
                    break
                else
                    From_index = Node_added_index - 1;
                    Num_extend = Num_extend + 1;
                end

            end

            if Success_num >= 1
                break
            end

        end
        %%%%���2�����ű߽����ͨ����չһ������
        if Num_node_to == 1
            Node_from = NODE_FROM;
            From_index = FROM_INDEX;

            Num_extend = 1;

            if  Type_point == 1 
                %��ʾ���ű߽���չ�����߸ս���ͨ��
                %�����ֻ��չһ���Ĳ��������ߵ���Ŀ���
                Num_extand_max = +1;   %20
                Node_type = 2;      %��ʾ���е������

            else
                if Type_point == 2
                    %��ʾ��խͨ���ڣ��򾡿�����չ��ֱ��������ײ
                    Num_extand_max = +1;  %20
                    Node_type = 3;      %��ʾ���е������
                else    % ��Type_point ==3
                    %��ʾ��խͨ���ڣ��򾡿�����չ��ֱ��������ײ
                    Num_extand_max = 20; %20
                    Node_type = 3;      %��ʾ���е������
                end
            end

            while true
                [Node, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                        Steer(Tree, From_index, Node_to, Step, Robot_size, Boundary, Obstacle, Num_collision_test, Weight);
                %Type_steer = 1, ǰ��һ��
                %Type_steer = 2�����������
                %Type_steer = 3, ������ײ

                if Judge == 0
                    Tree.Node(:, Node_added_index) = Node;
                    Tree.Cost(:, Node_added_index) = Cost;
                    Tree.Parent(Node_added_index) = From_index;
                    Tree.Children_num(From_index) = Tree.Children_num(From_index) + 1;
                    Tree.Children(Tree.Children_num(From_index), From_index) = Node_added_index;  
                    Tree.Type(:, Node_added_index) = Node_type;

                    Tree.Children_num(Node_added_index) = 0;
                    Tree.Children(1, Node_added_index) = 0; %��������ӽڵ������ʼ��

                    Node_added_index = Node_added_index + 1;

                    Dist_temp = Distance(Node, Node_goal, Weight);

                    if Dist_temp < Error_goal
                        Success_num = Success_num + 1;
                        Success(1, Success_num) = Node_added_index - 1;
                        Success(2, Success_num) = Dist_temp;

                        disp(['��' num2str(Success_num) '�γɹ��ҵ���' num2str(Success_num) '·������ ' num2str(toc) ' s']);
                        break;
                    end

                else
                    [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight, Error_goal);               
                    break;
                end

                if  Num_extend >= Num_extand_max || Type_steer == 2    
                    %����������չ����, ֱ��������ڲŻ�ֹͣ
                    break
                else
                    From_index = Node_added_index - 1;
                    Num_extend = Num_extend + 1;
                end

            end

            if Success_num >= 1
                break
            end

        end
        
        %%% �����ж��Ƿ���Ҫ��������
        if  Type_point == 3 && Judge ~= 0 && Num_extend ~= 1 && Num_continue < inf
            %�����ͨ���У����ٷ�����һ����չ���Ƿ�������ײ�������������չ        
            
        	IsContinue = true;
            Num_continue = Num_continue + 1;
        else
            IsContinue = false;
        end
        
    end
     
end

Total_node = Node_added_index - 1;          %�ҵ����ܽڵ���
Result = [Iter_num, Node_added_index-1, Num_collision_test, Num_collision_points, Num_PCA_compute_1, Num_PCA_compute_2, toc];
disp(['�ܹ������� ' num2str(Iter_num) ' �ε��������� ' num2str(Node_added_index-1) ' ���ڵ㣬���� ' num2str(toc) ' s']);
disp(['�ܹ������� ' num2str(Num_collision_test) ' ����ײ��⣬���� ' num2str(Num_collision_points) ' ����ײ��']);
disp(['�ܹ������� ' num2str(Num_PCA_compute_1) ' �ε�һ��PCA����, ' num2str(Num_PCA_compute_2) ' �εڶ���PCA����']);
disp(['�ܹ������� ' num2str(toc) ' s']);
 
%������
% Solution = TreePlot(Tree, Total_node, Node_init, Node_goal, Success_num, Success, Robot_size, Obstacle, Weight);         %������


end
