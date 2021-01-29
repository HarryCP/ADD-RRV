% ADD RRV 
% Copyrigt by Peng Cai 
% 01/29/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%每次拓展都将碰撞边界信息记录下来，
%如果在边界构型附近搜索到的已添加的碰撞点数量小于下界；
%表明此区域还没有被完全探索，
%因此利用基础的RRV来在拓展，
%主要是利用了RRV在寻找窄通道入口和在窄通道里的优势；
%但是单纯的RRV需要进行大量的随机采样和碰撞检测，计算量大，
%因此当在边界构型附近搜集到足够的碰撞点信息时，
%利用碰撞点指导拓展，同时能够防止拓展陷入非凸障碍物中
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function Result = Main()
% 地图为锯齿形
clear
clc

addpath('~\Long Passage\Supply Tools\Supply Tools\作图');
addpath('~\Long Passage\Supply Tools\Supply Tools\GJK Algorithm');
addpath('~\Long Passage\Supply Tools\Supply Tools\Transport');
%根据实际路径修改

%% 初始条件和参数设置
Boundary = [   0,  12;
               0, 8.5; 
             -pi,  pi];
% %地图边界

Node_init = [1, 5, pi/2]';   %初始位置
Node_goal = [11, 5, pi/2]';   %目标位置

delta = 0.7;        %机器人的大小1, 0.7

Robot_size = ShapeToPoint([0, 0]', delta*0.5, delta*0.1);   %机器人在原点时的状态

Weight = diag([1, 1, 1]); % 用于0.7大小下的权重 diag([1, 1, 0.5]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Obstacle = struct;  %障碍物参数
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
%其他参数
Max_iter = 1*10^5;                      %最大迭代次数
Step = 0.1;                             %机器人每一步步长
Error_goal = 0.01;                      %允许误差

P_sample = 0.3;                         %以目标点为采样点的概率
P_belif = 0.60;                         %置信区间范围，再[0, 1]内选取
P_bridge = 0.8;                         %利用PCA偏置桥测的概率  0.8

Radius = 5*Step;                        %搜索半径

Radius_ADDRRT = 30*Step;                %ADD-RRT初始半径 30
alpha = 0.05;                           %ADD-RRT自适应系数

Num_obstacle_limit = 100;                        %用于判断入口处的碰撞点是否足够丰富,100
Num_max_bridge_test = 20;               %最大桥测试次数

Num_collision_test = 0;                         %碰撞检测次数
Num_collision_points = 0;                       %遇到的碰撞点
Num_entrance_points = 0;                        %检测到的入口点
Num_narrow_points = 0;                          %表示在窄通道中的点
Num_PCA_compute_1 = 0;                          %RRV中利用PCA分析的次数
Num_PCA_compute_2 = 0;                          %利用边界点进行PCA分析的次数

%% 创建一棵树
Tree = struct;
Num_node = 1*10^4;
Dim = 3;
Tree.Node = zeros(Dim, Num_node );                                        	%表示节点
Tree.Cost = zeros(1, Num_node);                                             %表示从上一个节点到当前节点的代价
Tree.Parent = zeros(1, Num_node, 'int32');                                  %记录节点的父节点索引
Tree.Children = zeros(Num_node, Num_node, 'int32');                         %记录每个节点下的子节点索引
Tree.Children_num = zeros(1, Num_node, 'int32');                            %记录每个节点下的子节点数量
Tree.Type = zeros(1, Num_node);                                             %记录每个点的类型:0,普通构型；1入口构型；2沿边界构型；3窄通道内构型；4通过桥测试的构型 
Tree.ADD_Radius = 1./zeros(1, Num_node);                                    	%记录每个节点的动态半径

Tree.Node(:, 1) = Node_init;
Tree.ADD_Radius(:, 1) = +inf;

Node_added_index = 2;                               %当前添加的点的编号

Collision_boundary_set = [];            %用于记录发生碰撞边界点
Entrance_set = [];                      %用于记录检测到的入口点
Entrance_flag = [];                     %用于记录找到的入口是否已经被访问，0表示未访问，1表示访问
Narrow_set = [];                        %表示在窄通道中的点

Success_num = 0;                        %表示成功找到路径的次数
Success = zeros(2, Num_node);           %表示所找到的路径的最后一个节点的索引和距离

rng('shuffle');

tic;
% 开始计时

for Iter_num = 1:Max_iter

    if(mod(Iter_num, 100) == 0)
        disp([num2str(Iter_num) ' 次迭代，生成 ' num2str(Node_added_index-1) ' 个节点，花费 ' num2str(toc) ' s']);
    end
    
    
    while true
        Node_rand = RandomSample( P_sample, Boundary, Node_goal );
        %随机撒点    
    
        [Node_nearest_index, Dist_min] = Nearest(Tree, Node_rand, Node_added_index, Weight);
        %寻找最近的节点
        
        if Distance(Tree.Node(:, Node_nearest_index), Node_rand, Weight) < Tree.ADD_Radius(:, Node_nearest_index)
            break;
        end        
    end
    
    From_index = Node_nearest_index;
    
	Num_extend = 1; 
    Num_extand_max = 1; %表示最大拓展步数:1/+inf
    
    while true
        [Node, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                Steer(Tree, From_index, Node_rand, Step, Robot_size, Boundary, Obstacle, Num_collision_test, Weight);
     	%Type_steer = 1, 前进一步
        %Type_steer = 2，到达采样点
        %Type_steer = 3, 发生碰撞
        
        if Judge == 0
            Tree.Node(:, Node_added_index) = Node;
            Tree.Cost(:, Node_added_index) = Cost;
            Tree.Parent(Node_added_index) = From_index;
            Tree.Children_num(From_index) = Tree.Children_num(From_index) + 1;
            Tree.Children(Tree.Children_num(From_index), From_index) = Node_added_index;  
            Tree.Type(:, Node_added_index) = 0;
            Tree.ADD_Radius(Node_added_index) = +inf;
            
            Tree.ADD_Radius(From_index) = (1 + alpha) *  Tree.ADD_Radius(From_index);   %成功拓展，父节点半径扩展              
        
            Tree.Children_num(Node_added_index) = 0;
            Tree.Children(1, Node_added_index) = 0; %对自身的子节点情况初始化
        
            Node_added_index = Node_added_index + 1;
        
            Dist_temp = Distance(Node, Node_goal, Weight);
        
            if Dist_temp < Error_goal
                Success_num = Success_num + 1;
                Success(1, Success_num) = Node_added_index - 1;
                Success(2, Success_num) = Dist_temp;
            
                disp(['第' num2str(Success_num) '次成功找到第' num2str(Success_num) '路径，需 ' num2str(toc) ' s']);
                break;
            end
        
        else
            if Tree.ADD_Radius(From_index) == +inf
                Tree.ADD_Radius(From_index) = Radius_ADDRRT;
            else
                if Tree.ADD_Radius(From_index) > 2*Step  % 2
                    Tree.ADD_Radius(From_index) = (1 - alpha) * Tree.ADD_Radius(From_index);
                end
            end  
            %修改动态域
            
            [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight, Error_goal);
            break;
        end
        
        if  Num_extend >= Num_extand_max || Type_steer == 2    %用来控制拓展步长
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
    
    %% 下面开始利用获取的构型空间信息进行拓展
    if Type_steer ~= 3  %如果不发生碰撞，则下列步骤不继续
        continue
    end
    
    Type_point = 0;
    %表示关键构型点类型：
    %0 表示初始化；
    %1 表示PCA分析后沿着边界拓展
    %2 表示PCA分析后找到的可能入口
    %3 表示PCA分析后找到的窄通道
    
    Flag_new_entrance = 0;  %表示是否找到新的入口点：0表示否；1表示是
    num_bridgetest = 1;
    
    Node_from = Tree.Node(:, From_index);
    Points_obstacle = Near(Collision_boundary_set, Num_collision_points, Node_from, Radius, Weight);
    %搜集附近的碰撞构型   
	[~, Num_points_obstacle] = size(Points_obstacle);
    
    Node_rand = UnifyAttitude(Node_rand, Node_from);
	%根据中心点对姿态进行统一化 （向Node_from统一）
    
    Num_node_to = 0;    
    if Num_points_obstacle < Dim+1
        %如果接触构型（in-contact free configuration）附近的边界构型（obstacle boundary configuration）不够丰富 
        
        %%%%!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %多次进行桥测试
        
        while num_bridgetest > 0
            num_bridgetest = num_bridgetest - 1;
        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%首先进行桥测试%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Bridge_point_1 = State_collision;
            %桥第一个支点为相应的碰撞点
            Bridge_vect = 2*rand(Dim, 1) - 1; 
    %         Bridge_vect = BridgeTest(Node_rand - Node_from, Dim, Weight);
            %桥测试方向
            if rand < P_bridge %一定概率下，利用PCA偏置桥测试方向
                Points_free = Near(Tree.Node(:, 1:Node_added_index - 1), Node_added_index - 1, Node_from, Radius, Weight);
                %搜集附近的自由构型
                [~, Num_points_free] = size(Points_free);

                if Num_points_free >= Dim+1      %PCA偏置桥测试方向
                    [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight*Points_free)');
                    Num_PCA_compute_2 = Num_PCA_compute_2 + 1;

                    Bridge_vect = BridgeProject(Bridge_vect, Coeff_free, Latent_free, Dim);
                end                       
            end

            Bridge_length = Radius + (Radius/3)*randn;          
            %桥测试长度
            Bridge_point_2 = State_collision + Bridge_vect*Bridge_length/Distance(Bridge_vect, zeros(Dim, 1), Weight);
            %桥第二个支点
            Middle_point = (Bridge_point_1 + Bridge_point_2) /2;
            %桥中点

            Num_collision_test = Num_collision_test + 1;
            if IsCollision(Bridge_point_2, Robot_size, Boundary, Obstacle) ~= 0     %桥另一个点发生碰撞
                Num_collision_test = Num_collision_test + 1;
                if IsCollision(Middle_point, Robot_size, Boundary, Obstacle) == 0   %桥中点无碰撞
                    [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Middle_point, Num_entrance_points, Weight, Error_goal);                
                    if IsSucceedAdd == 1
                        Entrance_flag(:, Num_entrance_points) = 0;               
                        %记录可能的通道入口
                        Flag_new_entrance = 1;  %表示初次桥测试找到入口 
                        break
                    end               
                end       
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%第一次桥测试结束%%%%%%%%%%%%%%%%%%%%%%
        
        end
       %%%%!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
        
    else
        if Num_points_obstacle <= Num_obstacle_limit
            Num_PCA_compute_1  = Num_PCA_compute_1 +1;
            Points_free = Near(Tree.Node(:, 1:Node_added_index - 1), Node_added_index - 1, Node_from, Radius, Weight);
            %搜集附近的自由构型
            [~, Num_points_free] = size(Points_free);
        
            [Coeff_obs, Score_obs, Latent_obs, Tsquared_obs, Explained_obs, Mu_obs] = pca((Weight*Points_obstacle)');
            % Coeff为协方差矩阵的特性向量矩阵（列向量），变换矩阵
            % Score为变换后的点,Score = (X-Mu) * Coeff.
            % Latent为各特征向量对应的特征值，从大到小排列
            % Explained每一个主成分的贡献比
            % Mu为X（原数据）按列的均值
            Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
        
            [Points_free_in_ellipsoid, Judge_node_from_in_ellipsoid] = PointsAnalyse(Coeff_obs, Latent_obs, Mu_obs, Points_free, Num_points_free, Dim, Node_from, P_belif, Weight);
            %根据PCA对障碍物中的点的分析结果，对无碰撞的点进行再分析
            
            if isempty(Points_free_in_ellipsoid)
                %障碍物点包络区域内不包含无碰撞点
                %Node_from在凸障碍物边界
                
                Num_node_to = 1;
                Node_to = NodeProject(Node_from, Node_rand, Coeff_obs, Latent_obs);
                %对随机节点在最优拓展方向上进行投影
                Type_point = 1;  %表示PCA分析后沿着边界拓展

                if Num_points_obstacle <= Num_max_bridge_test
                    %%%%!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %多次进行桥测试
                    while num_bridgetest > 0
                        num_bridgetest = num_bridgetest - 1;

                    %%%%%%%%%%%%%%%%%%%%%%%%%%再次进行桥测试%%%%%%%%%%%%%%%%%%%%%%%
                                       
                        Bridge_point_1 = State_collision;
                        %桥第一个支点为相应的碰撞点
                        Bridge_vect = 2*rand(Dim,1) - 1;                            
    %                     Bridge_vect = BridgeTest(Node_rand - Node_from, Dim, Weight);
                        %桥测试方向
                        if rand < P_bridge %一定概率下，利用PCA偏置桥测试方向            
                            if Num_points_free >= Dim+1      %PCA偏置桥测试方向
                                [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight*Points_free)');
                                Num_PCA_compute_2 = Num_PCA_compute_2 + 1;                
                                Bridge_vect = BridgeProject(Bridge_vect, Coeff_free, Latent_free, Dim);
                            end
                        end

                        Bridge_length = Radius + (Radius/3)*randn;          
                        %桥测试长度
                        Bridge_point_2 = State_collision + Bridge_vect*Bridge_length/Distance(Bridge_vect, zeros(Dim, 1), Weight);
                        %桥第二个支点
                        Middle_point = (Bridge_point_1 + Bridge_point_2) /2;
                        %桥中点

                        Num_collision_test = Num_collision_test + 1;
                        if IsCollision(Bridge_point_2, Robot_size, Boundary, Obstacle) ~= 0     %桥另一个点发生碰撞
                            Num_collision_test = Num_collision_test + 1;
                            if IsCollision(Middle_point, Robot_size, Boundary, Obstacle) == 0   %桥中点无碰撞
                                [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Middle_point, Num_entrance_points, Weight, Error_goal);                
                                if IsSucceedAdd == 1
                                    Entrance_flag(:, Num_entrance_points) = 0;               
                                    %记录可能的通道入口
                                    Flag_new_entrance = 1;  %表示桥测试找到入口
                                    break
                                end               
                            end            
                        end   
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%第二次桥测试结束%%%%%%%%%%%%%%%%%%%%%%
                end
                %%%%%%%%%%%%!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
            else
                
                if Judge_node_from_in_ellipsoid == 0
                    % Node_from可能在窄通道入口处                    
                    
                    %%%%%%%添加入口节点%%%%%%%%%
                    [~, Num_temp] = size(Points_free_in_ellipsoid);
                    Points_center = sum(Points_free_in_ellipsoid, 2)/Num_temp;
                    
                    [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Points_center, Num_entrance_points, Weight, Error_goal);                
                    if IsSucceedAdd == 1
                        Entrance_flag(:, Num_entrance_points) = 0;               
                        %记录可能的通道入口                   
                        Flag_new_entrance = 1;  %表示找到入口                      
                    end                   
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%
%                   	Node_parent = Tree.Node(:, Tree.Parent(From_index));
%                     if (Weight*(Node_parent - Node_from))' * (Weight*(Node_rand - Node_from)) < 0
                        [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight*Points_free)');
                        Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
                        Num_node_to = 1;

                        Node_temp_goal = 100*(Node_rand - Node_from) + Node_from;    %将目标延长，使得树在通道中尽可能拓展
                        Node_to = NodeProject(Node_from, Node_temp_goal, Coeff_free, Latent_free);
                        %对随机节点在最优拓展方向上进行投影(自由空间的伸展方向)     
%                     end                          
                   	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    Type_point = 2;  %表示PCA分析后找到入口

                else
                    %Node_from在窄通道内（或者在非凸障碍物处）
                    %确定拓展方向
%                   	Node_parent = Tree.Node(:, Tree.Parent(From_index));
%                     if (Weight*(Node_parent - Node_from))' * (Weight*(Node_rand - Node_from))  < 0
                        [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight*Points_free)');
                        Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
                        Num_node_to = 1;
                    
                        Node_temp_goal = 100*(Node_rand - Node_from) + Node_from;    %将目标延长，使得树在通道中尽可能拓展
                        Node_to = NodeProject(Node_from, Node_temp_goal, Coeff_free, Latent_free);
                        Type_point = 3;  %表示PCA分析后，在通道内
%                     end
                end
            end
        end
    end
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%下面进行树的生长%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%先记录初始节点，便于多次拓展
    NODE_FROM = Node_from;
    FROM_INDEX = From_index;
       
    %%%%情况1：如果找到了入口处的节点，先对入口节点进行拓展
    if Flag_new_entrance == 1
    	Node_from = NODE_FROM;
        From_index = FROM_INDEX;
        
        Num_extend = 1;
        Num_extand_max = +inf;  %表示最大拓展步数
        while true
            [Node, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                    Steer(Tree, From_index, Entrance_set(:, Num_entrance_points), Step, Robot_size, Boundary, Obstacle, Num_collision_test, Weight);
            %Type_steer = 1, 前进一步
            %Type_steer = 2，到达采样点
            %Type_steer = 3, 发生碰撞
        
            if Judge == 0
                Tree.Node(:, Node_added_index) = Node;
                Tree.Cost(:, Node_added_index) = Cost;
                Tree.Parent(Node_added_index) = From_index;
                Tree.Children_num(From_index) = Tree.Children_num(From_index) + 1;
                Tree.Children(Tree.Children_num(From_index), From_index) = Node_added_index;  
                Tree.Type(:, Node_added_index) = 0;
                Tree.ADD_Radius(Node_added_index) = +inf;
        
                Tree.Children_num(Node_added_index) = 0;
        
                Tree.Children_num(Node_added_index) = 0;
                Tree.Children(1, Node_added_index) = 0; %对自身的子节点情况初始化
        
                Node_added_index = Node_added_index + 1;
        
                Dist_temp = Distance(Node, Node_goal, Weight);
        
                if Dist_temp < Error_goal
                    Success_num = Success_num + 1;
                    Success(1, Success_num) = Node_added_index - 1;
                    Success(2, Success_num) = Dist_temp;
            
                    disp(['第' num2str(Success_num) '次成功找到第' num2str(Success_num) '路径，需 ' num2str(toc) ' s']);
                    break;
                end
        
            else
                [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight, Error_goal);
                
                %如果碰撞构型就是目标入口构型，则目标入口构型标记为一探测状态
                if Distance(Entrance_set(:, Num_entrance_points), State_collision, Weight) < Error_goal
                    Entrance_flag(:, Num_entrance_points) = 1;
                end
                
                break;
            end
        
            if  Num_extend >= Num_extand_max || Type_steer == 2    
                %用来控制拓展步长, 直到到达入口才会停止
                Entrance_flag(:, Num_entrance_points) = 1;
                %表示入口集合中，此点已经被找到
                
                Tree.Type(:, Node_added_index-1) = 1; 
                %表示树中，新添加的点为入口点
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
    %%%%情况2：沿着边界或者通道拓展一定步数
    if Num_node_to == 1
    	Node_from = NODE_FROM;
        From_index = FROM_INDEX;
        
        Num_extend = 1;
        
        if  Type_point == 1 
            %表示沿着边界拓展，或者刚进入通道
            %则最大只拓展一定的步数，或者到达目标点
            Num_extand_max = +1;% 20
            Node_type = 2;      %表示树中点的类型
                       
        else
            if Type_point == 2
                %表示在窄通道内，则尽可能拓展，直到发生碰撞
                Num_extand_max = 20;    % 20
                Node_type = 3;      %表示树中点的类型
            else    % 即Type_point ==3
            	%表示在窄通道内，则尽可能拓展，直到发生碰撞
                Num_extand_max = 20;    % 20
                Node_type = 3;      %表示树中点的类型
            end
        end
        
        while true
            [Node, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                    Steer(Tree, From_index, Node_to, Step, Robot_size, Boundary, Obstacle, Num_collision_test, Weight);
            %Type_steer = 1, 前进一步
            %Type_steer = 2，到达采样点
            %Type_steer = 3, 发生碰撞
        
            if Judge == 0
                Tree.Node(:, Node_added_index) = Node;
                Tree.Cost(:, Node_added_index) = Cost;
                Tree.Parent(Node_added_index) = From_index;
                Tree.Children_num(From_index) = Tree.Children_num(From_index) + 1;
                Tree.Children(Tree.Children_num(From_index), From_index) = Node_added_index;  
                Tree.Type(:, Node_added_index) = Node_type;
                Tree.ADD_Radius(Node_added_index) = +inf;
        
                Tree.Children_num(Node_added_index) = 0;
                Tree.Children(1, Node_added_index) = 0; %对自身的子节点情况初始化
        
                Node_added_index = Node_added_index + 1;
        
                Dist_temp = Distance(Node, Node_goal, Weight);
        
                if Dist_temp < Error_goal
                    Success_num = Success_num + 1;
                    Success(1, Success_num) = Node_added_index - 1;
                    Success(2, Success_num) = Dist_temp;
            
                    disp(['第' num2str(Success_num) '次成功找到第' num2str(Success_num) '路径，需 ' num2str(toc) ' s']);
                    break;
                end
        
            else
                [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight, Error_goal);               
                break;
            end
        
            if  Num_extend >= Num_extand_max || Type_steer == 2    
                %用来控制拓展步长, 直到到达入口才会停止
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

Total_node = Node_added_index - 1;          %找到的总节点数
Result = [Iter_num, Node_added_index-1, Num_collision_test, Num_collision_points, Num_PCA_compute_1, Num_PCA_compute_2, toc];
disp(['总共进行了 ' num2str(Iter_num) ' 次迭代，生成 ' num2str(Node_added_index-1) ' 个节点，花费 ' num2str(toc) ' s']);
disp(['总共进行了 ' num2str(Num_collision_test) ' 次碰撞检测，生成 ' num2str(Num_collision_points) ' 个碰撞点']);
disp(['总共进行了 ' num2str(Num_PCA_compute_1) ' 次第一类PCA分析, ' num2str(Num_PCA_compute_2) ' 次第二类PCA分析']);
disp(['总共花费了 ' num2str(toc) ' s']);
 
%输出结果
Solution = TreePlot(Tree, Total_node, Node_init, Node_goal, Success_num, Success, Robot_size, Obstacle, Weight);         %画出树


% end
