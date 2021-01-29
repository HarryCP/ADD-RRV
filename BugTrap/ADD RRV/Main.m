% ADD RRV 
% Copyrigt by Peng Cai 
% 01/29/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ÿ����չ������ײ�߽���Ϣ��¼������
%����ڱ߽繹�͸���������������ӵ���ײ������С���½磻
%����������û�б���ȫ̽����
%��Ҫ��������RRV��Ѱ��խͨ����ں���խͨ��������ƣ�
%���ǵ�����RRV��Ҫ���д����������������ײ��⣬��������
%��˵��ڱ߽繹�͸����Ѽ����㹻����ײ����Ϣʱ��
%������ײ��ָ����չ��ͬʱ�ܹ���ֹ��չ�����͹�ϰ�����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function Result = Main()
%��ͼΪ�����Թ�
clear
clc

addpath('~\BugTrap\Supply Tools\��ͼ');
addpath('~\BugTrap\Supply Tools\GJK Algorithm');
%·����Ҫ����ʵ����������޸�

%% ��ʼ�����Ͳ�������
Boundary = [ 0 10;
             0 10 ];
%��ͼ�߽�

Node_init = [7, 3.5]';      %��ʼλ��
% Node_goal = [0.5, 6.5]';    %Ŀ��λ��
Node_goal = [9.5, 6.5]';    %Ŀ��λ��

Robot_size = 0.1;   %������Բ�ΰ뾶

Obstacle = struct;  %�ϰ������
Obstacle.NO1 = ShapeToPoint([1.3, 5]', 0.6, 5);
Obstacle.NO2 = ShapeToPoint([5, 7.2]', 8, 0.6);
Obstacle.NO3 = ShapeToPoint([5, 2.8]', 8, 0.6);
Obstacle.NO4 = ShapeToPoint([8.7, 6.4]', 0.6, 2.2);
Obstacle.NO5 = ShapeToPoint([8.7, 3.6]', 0.6, 2.2);
Obstacle.NO6 = ShapeToPoint([6.5, 4.5]', 5, 0.6);
Obstacle.NO7 = ShapeToPoint([6.5, 5.5]', 5, 0.6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��������
Max_iter = 5*10^4;                      %����������
Step = 0.1;                             %������ÿһ������
Error_goal = 0.01;                      %�������
Dim = 2;                                %���Ϳռ�ά��

P_sample = 0.3;                         %��Ŀ���Ϊ������ĸ��� 0.3
P_belif = 0.60;                         %�������䷶Χ����[0, 1]��ѡȡ
P_bridge = 0.8;                         %����PCAƫ���Ų�ĸ��� 

Radius = 5*Step;                        %�����뾶

Radius_ADDRRT = 30*Step;                %ADD-RRT��ʼ�뾶 30
alpha = 0.05;                           %ADD-RRT����Ӧϵ��

Num_obstacle_limit = 15;                %�����ж���ڴ�����ײ���Ƿ��㹻�ḻ,30
Num_max_bridge_test = Dim+1;            %����Ų��Դ���

%%%%%%
Num_collision_test = 0;                         %��ײ������
Num_collision_points = 0;                       %��������ײ��
Num_entrance_points = 0;                        %��⵽����ڵ�
Num_narrow_points = 0;                          %��ʾ��խͨ���еĵ�
Num_PCA_compute_1 = 0;                          %RRV������PCA�����Ĵ���
Num_PCA_compute_2 = 0;                          %���ñ߽�����PCA�����Ĵ���

%% ����һ����
Tree = struct;
Num_node = 1*10^4;                                                                   %��ʾ���Ϳռ�ά��
Tree.Node = zeros(Dim, Num_node);                                          	%��ʾ�ڵ�
Tree.Cost = zeros(1, Num_node);                                             %��ʾ����һ���ڵ㵽��ǰ�ڵ�Ĵ���
Tree.Parent = zeros(1, Num_node, 'int32');                                  %��¼�ڵ�ĸ��ڵ�����
Tree.Children = zeros(Num_node, Num_node, 'int32');                         %��¼ÿ���ڵ��µ��ӽڵ�����
Tree.Children_num = zeros(1, Num_node, 'int32');                            %��¼ÿ���ڵ��µ��ӽڵ�����
Tree.Type = zeros(1, Num_node);                                             %��¼ÿ���������:0,��ͨ���ͣ�1��ڹ��ͣ�2�ر߽繹�ͣ�3խͨ���ڹ��ͣ�4ͨ���Ų��ԵĹ���
Tree.ADD_Radius = 1./zeros(1, Num_node);                                 	%��¼ÿ���ڵ�Ķ�̬�뾶

Tree.Node(:, 1) = Node_init;
Tree.ADD_Radius(:, 1) = +inf;

Node_added_index = 2;                  	%��ǰ��ӵĵ�ı��

Collision_boundary_set = [];            %���ڼ�¼������ײ�߽��
Entrance_set = [];                      %���ڼ�¼��⵽����ڵ�
Entrance_flag = [];                     %���ڼ�¼�ҵ�������Ƿ��Ѿ������ʣ�0��ʾδ���ʣ�1��ʾ����
Narrow_set = [];                        %��ʾ��խͨ���еĵ�

Success_num = 0;                        %��ʾ�ɹ��ҵ�·���Ĵ���
Success = zeros(2, Num_node);           %��ʾ���ҵ���·�������һ���ڵ�������;���

%% RRT��������
rng('shuffle');

tic;
% ��ʼ��ʱ
kk = 0;
for Iter_num = 1:Max_iter

    if(mod(Iter_num, 100) == 0)
        disp([num2str(Iter_num) ' �ε��������� ' num2str(Node_added_index-1) ' ���ڵ㣬���� ' num2str(toc) ' s']);
    end
    	
    while true
        Node_rand = RandomSample( P_sample, Boundary, Node_goal );
        %�������
    
        [Node_nearest_index, Dist_min] = Nearest(Tree, Node_rand, Node_added_index);
        %Ѱ������Ľڵ�
       
        if Distance(Tree.Node(:, Node_nearest_index), Node_rand) < Tree.ADD_Radius(:, Node_nearest_index)
            if Tree.ADD_Radius(:, Node_nearest_index) == +inf %&& rand < 0.5
                if Tree.Children_num(:, Node_nearest_index) > Dim
                    continue;
                    %�Կտ�����ĵ���й���
                end
            end 
            break;
        end
    end
    
    From_index = Node_nearest_index;
    
    Num_extend = 1; 
    Num_extand_max = 1; %��ʾ�����չ����:1/+inf
    while true
        [Node, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                Steer(Tree, From_index, Node_rand, Step, Robot_size, Boundary, Obstacle, Num_collision_test);
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
            Tree.ADD_Radius(Node_added_index) = +inf;
            
            Tree.ADD_Radius(From_index) = (1 + alpha) *  Tree.ADD_Radius(From_index);   %�ɹ���չ�����ڵ�뾶��չ
        
            Tree.Children_num(Node_added_index) = 0;
            Tree.Children(1, Node_added_index) = 0; %��������ӽڵ������ʼ��
        
            Node_added_index = Node_added_index + 1;
        
            Dist_temp = Distance(Node, Node_goal);
        
            if Dist_temp < Error_goal
                Success_num = Success_num + 1;
                Success(1, Success_num) = Node_added_index - 1;
                Success(2, Success_num) = Dist_temp;
            
                disp(['��' num2str(Success_num) '�γɹ��ҵ���' num2str(Success_num) '·������ ' num2str(toc) ' s']);
                break;
            end
        
        else
            if Tree.ADD_Radius(From_index) == +inf
                Tree.ADD_Radius(From_index) = Radius_ADDRRT;
            else
                if Tree.ADD_Radius(From_index) > 2*Step
                    Tree.ADD_Radius(From_index) = (1 - alpha) * Tree.ADD_Radius(From_index);
                end
            end  
            %�޸Ķ�̬��
            [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Error_goal);
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
    
    continue
    
    %% ���濪ʼ���û�ȡ�Ĺ��Ϳռ���Ϣ������չ
    if Type_steer ~= 3  %�����������ײ�������в��費����
        continue
    end
    
    Type_point = 0;
    %��ʾ�ؼ����͵����ͣ�
    %0 ��ʾ��ʼ����
    %1 ��ʾPCA���������ű߽���չ
    %2 ��ʾPCA�������ҵ��Ŀ������
    %3 ��ʾPCA�������ҵ���խͨ��
    
    Flag_new_entrance = 0;  %��ʾ�Ƿ��ҵ��µ���ڵ㣺0��ʾ��1��ʾ��
    num_bridgetest = 1;
    
    Node_from = Tree.Node(:, From_index);
    Points_obstacle = Near(Collision_boundary_set, Num_collision_points, Node_from, Radius);
    %�Ѽ���������ײ����   
	[~, Num_points_obstacle] = size(Points_obstacle);
      
    Num_node_to = 0;
    if Num_points_obstacle < Dim+1 %Dim +1 
        %����Ӵ����ͣ�in-contact free configuration�������ı߽繹�ͣ�obstacle boundary configuration�������ḻ
        
        %%%%!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %��ν����Ų���
        
        while num_bridgetest > 0
            num_bridgetest = num_bridgetest - 1;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%���Ƚ����Ų���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Bridge_point_1 = State_collision;
            %�ŵ�һ��֧��Ϊ��Ӧ����ײ��
            Bridge_vect = 2*rand(Dim, 1) - 1;                            
            %�Ų��Է���
            if rand < P_bridge %һ�������£�����PCAƫ���Ų��Է���
                Points_free = Near(Tree.Node(:, 1:Node_added_index - 1), Node_added_index - 1, Node_from, Radius);
                %�Ѽ����������ɹ���
                [~, Num_points_free] = size(Points_free);

                if Num_points_free >= Dim+1      %PCAƫ���Ų��Է���
                    [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca(Points_free');
                    Num_PCA_compute_2 = Num_PCA_compute_2 + 1;

                    Bridge_vect = BridgeProject(Bridge_vect, Coeff_free, Latent_free, Dim);
                end                       
            end

            Bridge_length = Radius + (Radius/3)*randn;          
            %�Ų��Գ���
            Bridge_point_2 = State_collision + Bridge_vect*Bridge_length/Distance(Bridge_vect, zeros(2, 1));
            %�ŵڶ���֧��
            Middle_point = (Bridge_point_1 + Bridge_point_2) /2;
            %���е�

            Num_collision_test = Num_collision_test + 1;
            if IsCollision(Bridge_point_2, Robot_size, Boundary, Obstacle) ~= 0     %����һ���㷢����ײ
                Num_collision_test = Num_collision_test + 1;
                if IsCollision(Middle_point, Robot_size, Boundary, Obstacle) == 0   %���е�����ײ
                    [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Middle_point, Num_entrance_points, Error_goal);                
                    if IsSucceedAdd == 1
                        Entrance_flag(:, Num_entrance_points) = 0;               
                        %��¼���ܵ�ͨ�����
                        Flag_new_entrance = 1;  %��ʾ�����Ų����ҵ����  
                        break
                    end               
                end       
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%��һ���Ų��Խ���%%%%%%%%%%%%%%%%%%%%%%%
        end
        %%%%%%%!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    else
        if Num_points_obstacle <= Num_obstacle_limit
            Num_PCA_compute_1 = Num_PCA_compute_1 + 1;
            Points_free = Near(Tree.Node(:, 1:Node_added_index - 1), Node_added_index - 1, Node_from, Radius);
            %�Ѽ����������ɹ���
            [~, Num_points_free] = size(Points_free);
        
            [Coeff_obs, Score_obs, Latent_obs, Tsquared_obs, Explained_obs, Mu_obs] = pca(Points_obstacle');
            % CoeffΪЭ�������������������������������任����
            % ScoreΪ�任��ĵ�,Score = (X-Mu) * Coeff.
            % LatentΪ������������Ӧ������ֵ���Ӵ�С����
            % Explainedÿһ�����ɷֵĹ��ױ�
            % MuΪX��ԭ���ݣ����еľ�ֵ
            Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
        
            [Points_free_in_ellipsoid, Judge_node_from_in_ellipsoid] = PointsAnalyse(Coeff_obs, Latent_obs, Mu_obs, Points_free, Num_points_free, Dim, Node_from, P_belif);
            %����PCA���ϰ����еĵ�ķ��������������ײ�ĵ�����ٷ���
        
            if isempty(Points_free_in_ellipsoid)
                %�ϰ������������ڲ���������ײ��
                %Node_from��͹�ϰ���߽�
                
                Num_node_to = 1;
                Node_to = NodeProject(Node_from, Node_rand, Coeff_obs, Latent_obs);
                %������ڵ���������չ�����Ͻ���ͶӰ
                Type_point = 1;  %��ʾPCA���������ű߽���չ
                 
                if Num_points_obstacle <= Num_max_bridge_test
                    %%%%!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %��ν����Ų���

                    while num_bridgetest > 0
                        num_bridgetest = num_bridgetest - 1;
                
                    %%%%%%%%%%%%%%%%%%%%%%%%%%�ٴν����Ų���%%%%%%%%%%%%%%%%%%%%%%%
                    
                        Bridge_point_1 = State_collision;
                        %�ŵ�һ��֧��Ϊ��Ӧ����ײ��
                        Bridge_vect = 2*rand(2,1) - 1;                            
                        %�Ų��Է���
                        if rand < P_bridge %һ�������£�����PCAƫ���Ų��Է���            
                            if Num_points_free >= Dim+1      %PCAƫ���Ų��Է���
                                [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca(Points_free');
                                Num_PCA_compute_2 = Num_PCA_compute_2 + 1;                
                                Bridge_vect = BridgeProject(Bridge_vect, Coeff_free, Latent_free, Dim);
                            end
                        end

                        Bridge_length = Radius + (Radius/3)*randn;          
                        %�Ų��Գ���
                        Bridge_point_2 = State_collision + Bridge_vect*Bridge_length/Distance(Bridge_vect, zeros(2, 1));
                        %�ŵڶ���֧��
                        Middle_point = (Bridge_point_1 + Bridge_point_2) /2;
                        %���е�

                        Num_collision_test = Num_collision_test + 1;
                        if IsCollision(Bridge_point_2, Robot_size, Boundary, Obstacle) ~= 0     %����һ���㷢����ײ
                            Num_collision_test = Num_collision_test + 1;
                            if IsCollision(Middle_point, Robot_size, Boundary, Obstacle) == 0   %���е�����ײ
                                [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Middle_point, Num_entrance_points, Error_goal);                
                                if IsSucceedAdd == 1
                                    Entrance_flag(:, Num_entrance_points) = 0;               
                                    %��¼���ܵ�ͨ�����
                                    Flag_new_entrance = 1;  %��ʾ�Ų����ҵ����
                                    break;
                                end               
                            end            
                        end   
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%�ڶ����Ų��Խ���%%%%%%%%%%%%%%%%%%%%%%
                    
                end
                %%%%!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
            else
%                 continue;

                if Judge_node_from_in_ellipsoid == 0
                    % Node_from������խͨ����ڴ�                    
                    
                    %%%%%%%�����ڽڵ�%%%%%%%%%
                    [~, Num_temp] = size(Points_free_in_ellipsoid);
                    Points_center = sum(Points_free_in_ellipsoid, 2)/Num_temp;
                    
                    [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Points_center, Num_entrance_points, Error_goal);                
                    if IsSucceedAdd == 1
                        Entrance_flag(:, Num_entrance_points) = 0;               
                        %��¼���ܵ�ͨ�����                   
                        Flag_new_entrance = 1;  %��ʾ�ҵ����
                        
                    	%%%%%%%%%%%%%%%%%%%%%%%%                    
                        Node_parent = Tree.Node(:, Tree.Parent(From_index));
                        if (Node_parent - Node_from)' * (Node_rand - Node_from) < 0
                            [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca(Points_free_in_ellipsoid');
                            Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
                            Num_node_to = 1;
                            Node_temp_goal = 100*(Node_rand - Node_from) + Node_from;    %��Ŀ���ӳ���ʹ������ͨ���о�������չ
                            Node_to = NodeProject(Node_from, Node_temp_goal, Coeff_free, Latent_free);
                        end
                        %������ڵ���������չ�����Ͻ���ͶӰ(���ɿռ����չ����)                            
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    end                    


                    Type_point = 2;  %��ʾPCA�������ҵ����

                else
                    %Node_from��խͨ���ڣ������ڷ�͹�ϰ��ﴦ��
                    %ȷ����չ����
                    Node_parent = Tree.Node(:, Tree.Parent(From_index));
                    if (Node_parent - Node_from)' * (Node_rand - Node_from) < 0
                        [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca(Points_free_in_ellipsoid');
                        Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
                        Num_node_to = 1;
                        Node_temp_goal = 100*(Node_rand - Node_from) + Node_from;    %��Ŀ���ӳ���ʹ������ͨ���о�������չ
                        Node_to = NodeProject(Node_from, Node_temp_goal, Coeff_free, Latent_free);
                        Type_point = 3;  %��ʾPCA��������ͨ����       
                    end
                end            
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
                    Steer(Tree, From_index, Entrance_set(:, Num_entrance_points), Step, Robot_size, Boundary, Obstacle, Num_collision_test);
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
                Tree.ADD_Radius(Node_added_index) = +inf;
        
                Tree.Children_num(Node_added_index) = 0;
                Tree.Children(1, Node_added_index) = 0; %��������ӽڵ������ʼ��
        
                Node_added_index = Node_added_index + 1;
        
                Dist_temp = Distance(Node, Node_goal);
        
                if Dist_temp < Error_goal
                    Success_num = Success_num + 1;
                    Success(1, Success_num) = Node_added_index - 1;
                    Success(2, Success_num) = Dist_temp;
            
                    disp(['��' num2str(Success_num) '�γɹ��ҵ���' num2str(Success_num) '·������ ' num2str(toc) ' s']);
                    break;
                end
        
            else
                [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Error_goal);
                
                %�����ײ���;���Ŀ����ڹ��ͣ���Ŀ����ڹ��ͱ��Ϊһ̽��״̬
                if Distance(Entrance_set(:, Num_entrance_points), State_collision) < Error_goal
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
            Num_extand_max = +1;
            Node_type = 2;      %��ʾ���е������
                       
        else
            if Type_point == 2
                %��ʾ��խͨ���ڣ��򾡿�����չ��ֱ��������ײ
                Num_extand_max = +inf;  %+inf/20
                Node_type = 3;      %��ʾ���е������
            else    % ��Type_point ==3
            	%��ʾ��խͨ���ڣ��򾡿�����չ��ֱ��������ײ
                Num_extand_max = +inf;
                Node_type = 3;      %��ʾ���е������
            end
        end
        
        while true
            [Node, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                    Steer(Tree, From_index, Node_to, Step, Robot_size, Boundary, Obstacle, Num_collision_test);
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
                Tree.ADD_Radius(Node_added_index) = +inf;
        
                Tree.Children_num(Node_added_index) = 0;
                Tree.Children(1, Node_added_index) = 0; %��������ӽڵ������ʼ��
        
                Node_added_index = Node_added_index + 1;
        
                Dist_temp = Distance(Node, Node_goal);
        
                if Dist_temp < Error_goal
                    Success_num = Success_num + 1;
                    Success(1, Success_num) = Node_added_index - 1;
                    Success(2, Success_num) = Dist_temp;
            
                    disp(['��' num2str(Success_num) '�γɹ��ҵ���' num2str(Success_num) '·������ ' num2str(toc) ' s']);
                    break;
                end
        
            else
                [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Error_goal);               
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
    
    
end

Total_node = Node_added_index - 1;          %�ҵ����ܽڵ���
Result = [Iter_num, Node_added_index-1, Num_collision_test, Num_collision_points, Num_PCA_compute_1, Num_PCA_compute_2, toc];
disp(['�ܹ������� ' num2str(Iter_num) ' �ε��������� ' num2str(Node_added_index-1) ' ���ڵ㣬���� ' num2str(toc) ' s']);
disp(['�ܹ������� ' num2str(Num_collision_test) ' ����ײ��⣬���� ' num2str(Num_collision_points) ' ����ײ��']);
disp(['�ܹ������� ' num2str(Num_PCA_compute_1) ' �ε�һ��PCA����,�� ' num2str(Num_PCA_compute_2) ' �ε�һ��PCA����']);
disp(['�ܹ������� ' num2str(toc) ' s']);

% ������
Solution = TreePlot(Tree, Total_node, Node_init, Node_goal, Success_num, Success, Obstacle);         %������

% end


