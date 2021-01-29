% ADD RRV 
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
% function Result = Main()
%��ͼΪ�̶�������е��

 clear
 clc

addpath('~\Long Passage\Supply Tools\Supply Tools\Dynamics');
addpath('~\Long Passage\Supply Tools\Supply Tools\GJK Algorithm');
addpath('~\Long Passage\Supply Tools\Supply Tools\Transport');
addpath('~\Long Passage\Supply Tools\Supply Tools\��ͼ');
%����ʵ��·���޸�

%% ���ó�ʼ����
L1 = 1;      	%m�����˳���
L2 = 1;      	%m�����˳���
L3 = 1;     	%m�����˳���
L4 = 1;         %m�����˳���
L5 = 1;         %m�����˳���
L6 = 1;         %m�����˳���

W1 = 0.1;     	%m�����˿��
W2 = 0.1;     	%m�����˿��
W3 = 0.1;     	%m�����˿��
W4 = 0.1;     	%m�����˿��
W5 = 0.1;     	%m�����˿��
W6 = 0.1;     	%m�����˿��
%ϵͳ����

%�������˲�����װ�����ڴ���
Robot_parameter = struct;
Robot_parameter.L1 = L1;
Robot_parameter.L2 = L2;
Robot_parameter.L3 = L3;
Robot_parameter.L4 = L4;
Robot_parameter.L5 = L5;
Robot_parameter.L6 = L6;

Robot_parameter.W1 = W1;
Robot_parameter.W2 = W2;
Robot_parameter.W3 = W3;
Robot_parameter.W4 = W4;
Robot_parameter.W5 = W5;
Robot_parameter.W6 = W6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%״̬�߽�����
Boundary = struct;
Boundary.q1 = [-pi, pi];
Boundary.q2 = [-pi, pi];
Boundary.q3 = [-pi, pi];
Boundary.q4 = [-pi, pi];
Boundary.q5 = [-pi, pi];
Boundary.q6 = [-pi, pi];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��ʼ״̬
q1_init = pi/4;
q2_init = -pi/6;
q3_init = pi/6;
q4_init = pi/6;
q5_init = -pi/3;
q6_init = pi/4; 

q1_goal = 3*pi/4;
q2_goal = pi/6;
q3_goal = -pi/6;
q4_goal = -pi/6;
q5_goal = pi/3;
q6_goal = -pi/4; 

State_goal = [q1_init, q2_init, q3_init, q4_init, q5_init, q6_init]';
% State_goal = [q1_goal, q2_goal, q3_goal, q4_goal, q5_goal, q6_goal]';
% State_init = zeros(6, 1);   %����
State_init = [pi, 0, 0, 0, 0, 0]';   %����
End_goal = ForwardKine(State_goal, Robot_parameter);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%���ν�ģ%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�����˲���
%�����˲���
RobotData = struct;
RobotData.Body_num = 6;                 %�����˿��Էֽ�Ϊ6��͹�����

RobotData.Link1_C = ShapeToPoint([0, 0]', L1, W1);
%����������1����������������ϵ�µ����� 

RobotData.Link2_C = ShapeToPoint([0, 0]', L2, W2);
%����������2����������������ϵ�µ����� 

RobotData.Link3_C = ShapeToPoint([0, 0]', L3, W3);
%����������3����������������ϵ�µ����� 

RobotData.Link4_C = ShapeToPoint([0, 0]', L4, W4);
%����������4����������������ϵ�µ����� 

RobotData.Link5_C = ShapeToPoint([0, 0]', L5, W5);
%����������5����������������ϵ�µ����� 

RobotData.Link6_C = ShapeToPoint([0, 0]', L6, W6);
%����������6����������������ϵ�µ����� 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�ϰ������
ObstacleData = struct;
ObstacleData.NO1 = ShapeToPoint([5, 7]', 3, 0.5);
ObstacleData.NO2 = ShapeToPoint([-1, 7]', 10, 0.5);
ObstacleData.NO3 = ShapeToPoint([11, 7]', 6, 0.5);
% ObstacleData.NO4 = ShapeToPoint([5, 3]', 20, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%��������
Max_iter = 1*10^5;                          %����������
Step = 0.1;                                 %ÿ����С����չ����
Error_goal = 0.01;                          %Ŀ���������
Dim = 6;                                    %���Ϳռ�ά��

P_sample = 0.3;                             %��Ŀ���Ϊ������ĸ��� 0.3
P_belief = 0.60;                           	%�������䷶Χ����[0, 1]��ѡȡ
P_bridge = 0.8;                             %����PCAƫ���Ų�ĸ��� 0.8

Radius = 5*Step;                            %�����뾶

Radius_ADDRRT = 100*Step;                   %ADD-RRT��ʼ�뾶
alpha = 0.05;                               %ADD-RRT����Ӧϵ��

Num_obstacle_limit = 120;                  	%�����ж���ڴ�����ײ���Ƿ��㹻�ḻ,120
Num_max_bridge_test = 20;	%����Ų��Դ���

Weight_1 = diag([1, 1, 1, 1, 1, 1]);        %���ڹ��Ϳռ������Ȩ��
Weight_2 = diag([1, 1, 3]);              	%���ڹ����ռ����

%%%%%%
Num_collision_test = 0;                         %��ײ������
Num_collision_points = 0;                       %��������ײ��
Num_entrance_points = 0;                        %��⵽����ڵ�
Num_narrow_points = 0;                          %��ʾ��խͨ���еĵ�
Num_PCA_compute_1 = 0;                          %RRV������PCA�����Ĵ���
Num_PCA_compute_2 = 0;                          %���ñ߽�����PCA�����Ĵ���

%% ����һ�����������ṹ�壩
Tree = struct;
Num_node = 1*10^4;
Tree.State = zeros(Dim, Num_node);                                          %״̬�ռ�ڵ�
Tree.End = zeros(3, Num_node);                                              %ĩ��״̬�ڵ�
%(��������ʽ)
Tree.Cost =  zeros(1, Num_node);                                            %��ʾ����һ���ڵ㵽��ǰ�ڵ�Ĵ���
Tree.Parent = zeros(1, Num_node, 'int32');                                  %��¼�ڵ�ĸ��ڵ�����
Tree.Children = zeros(Num_node, Num_node, 'int32');                         %��¼ÿ���ڵ��µ��ӽڵ�����
Tree.Children_num = zeros(1, Num_node, 'int32');                            %��¼ÿ���ڵ��µ��ӽڵ�����
Tree.Type = zeros(1, Num_node); 
Tree.ADD_Radius = 1./zeros(1, Num_node); 

Tree.State(:, 1) = State_init;
Tree.End(:, 1) = ForwardKine(State_init, Robot_parameter);
Tree.ADD_Radius(:, 1) = +inf;
%�ڵ��ʼ��

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Node_added_index = 2;                   %��ǰ��ӵĵ�ı��

Collision_boundary_set = [];            %���ڼ�¼������ײ�߽��
Entrance_set = [];                      %���ڼ�¼��⵽����ڵ�
Entrance_flag = [];                     %���ڼ�¼�ҵ�������Ƿ��Ѿ������ʣ�0��ʾδ���ʣ�1��ʾ����
Narrow_set = [];                        %��ʾ��խͨ���еĵ�

Success_num = 0;                        %��ʾ�ɹ��ҵ�·���Ĵ���
Success = zeros(2, Num_node);           %��ʾ���ҵ���·�������һ���ڵ�������;���

%% ���������ڲ�������������
rng('shuffle');
% %���õ�λ���ڵĲ�������
% Seed_num = 500000;   %���Ӹ���
% Seed_unit_ball = SampleUnitBall(Dim, Seed_num);

%% RRT��������
tic;
% ��ʼ��ʱ
for Iter_num = 1:Max_iter
    
    if(mod(Iter_num, 100) == 0)
        disp([num2str(Iter_num) ' �ε��������� ' num2str(Node_added_index-1) ' ���ڵ㣬���� ' num2str(toc) ' s']);
    end   
    
    while true
        State_rand = RandomSample(P_sample, Boundary, State_goal);
        %�������
        End_rand = ForwardKine(State_rand, Robot_parameter);
    
        [Node_nearest_index, Dist_min] = Nearest(Tree, End_rand, Node_added_index, Weight_2);
        %Ѱ������Ľڵ�    
        
        if Distance(Tree.End(:, Node_nearest_index), End_rand, Weight_2, 1) < Tree.ADD_Radius(:, Node_nearest_index)
            break;
        end
    end  
    
	From_index = Node_nearest_index;

    Num_extend = 1; 
    Num_extand_max = 1; %��ʾ�����չ����:1/+inf
    
    while  true
        [State, End, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                Steer(Tree, From_index, State_rand, Step, Boundary, RobotData, ObstacleData, Robot_parameter, Num_collision_test, Weight_1, Error_goal);
     	%Type_steer = 1, ǰ��һ��
        %Type_steer = 2�����������
        %Type_steer = 3, ������ײ
        
        if Judge == 0
        	Tree.State(:, Node_added_index) = State;
        	Tree.End (:, Node_added_index) = End;
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

         	Dist_temp = Distance(End, End_goal, Weight_2, 2);
          	if Dist_temp < Error_goal*10
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
                if Tree.ADD_Radius(From_index) > 10*Step
                    Tree.ADD_Radius(From_index) = (1 - alpha) * Tree.ADD_Radius(From_index);
                end
            end  
            %�޸Ķ�̬��
            
            [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight_1, Error_goal);
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
    
%     IsContinue = true;
%     Num_continue = 0;
%     while IsContinue

        Type_point = 0;
        %��ʾ�ؼ����͵����ͣ�
        %0 ��ʾ��ʼ����
        %1 ��ʾPCA���������ű߽���չ
        %2 ��ʾPCA�������ҵ��Ŀ������
        %3 ��ʾPCA�������ҵ���խͨ��

        Flag_new_entrance = 0;  %��ʾ�Ƿ��ҵ��µ���ڵ㣺0��ʾ��1��ʾ��
        num_bridgetest = 120;
        
        
        State_from = Tree.State(:, From_index);
        
        Points_obstacle = Near(Collision_boundary_set, Num_collision_points, State_from, Radius, Weight_1);
        %�Ѽ���������ײ����   
        [~, Num_points_obstacle] = size(Points_obstacle);

        Num_node_to = 0;

        if Num_points_obstacle < Dim+1
        
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
                    Points_free = Near(Tree.State(:, 1:Node_added_index - 1), Node_added_index - 1, State_from, Radius, Weight_1);
                    %�Ѽ����������ɹ���
                    [~, Num_points_free] = size(Points_free);

                    if Num_points_free >= Dim+1      %PCAƫ���Ų��Է���
                        [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight_1*Points_free)');
                        Num_PCA_compute_2 = Num_PCA_compute_2 + 1;

                        Bridge_vect = BridgeProject(Bridge_vect, Coeff_free, Latent_free, Dim);
                    end                       
                end

                Bridge_length = Radius + (Radius/3)*randn;          
                %�Ų��Գ���
                Bridge_point_2 = State_collision + Bridge_vect*Bridge_length/Distance(Bridge_vect, zeros(Dim, 1), Weight_1, 1);
                %�ŵڶ���֧��
                Middle_point = (Bridge_point_1 + Bridge_point_2) /2;
                %���е�

                Num_collision_test = Num_collision_test + 1;
                if IsCollision(Bridge_point_2, Boundary, RobotData, ObstacleData, Robot_parameter) ~= 0     %����һ���㷢����ײ
                    Num_collision_test = Num_collision_test + 1;
                    if IsCollision(Middle_point, Boundary, RobotData, ObstacleData, Robot_parameter) == 0   %���е�����ײ
                        [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Middle_point, Num_entrance_points, Weight_1, Error_goal);                
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
            %%%%%%%%%%%!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        else
            if Num_points_obstacle <= Num_obstacle_limit
                Points_free = Near(Tree.State(:, 1:Node_added_index - 1), Node_added_index - 1, State_from, Radius, Weight_1);
                %�Ѽ����������ɹ���
                [~, Num_points_free] = size(Points_free);

                [Coeff_obs, Score_obs, Latent_obs, Tsquared_obs, Explained_obs, Mu_obs] = pca((Weight_1*Points_obstacle)');
                % CoeffΪЭ�������������������������������任����
                % ScoreΪ�任��ĵ�,Score = (X-Mu) * Coeff.
                % LatentΪ������������Ӧ������ֵ���Ӵ�С����
                % Explainedÿһ�����ɷֵĹ��ױ�
                % MuΪX��ԭ���ݣ����еľ�ֵ
                Num_PCA_compute_2 = Num_PCA_compute_2 + 1;

                [Points_free_in_ellipsoid, Judge_node_from_in_ellipsoid] = PointsAnalyse(Coeff_obs, Latent_obs, Mu_obs, Points_free, Num_points_free, Dim, State_from, P_belief, Weight_1);
                %����PCA���ϰ����еĵ�ķ��������������ײ�ĵ�����ٷ���

                if isempty(Points_free_in_ellipsoid)
                    %�ϰ������������ڲ���������ײ��
                    %Node_from��͹�ϰ���߽�

                    Num_node_to = 1;
                    State_to = NodeProject(State_from, State_rand, Coeff_obs, Latent_obs);
                    %������ڵ���������չ�����Ͻ���ͶӰ
                    Type_point = 1;  %��ʾPCA���������ű߽���չ
                    
                    
                    %%%%!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %��ν����Ų���
                    if Num_points_obstacle <= Num_max_bridge_test
                        while num_bridgetest > 0
                            num_bridgetest = num_bridgetest - 1;
                    
                        %%%%%%%%%%%%%%%%%%%%%%%%%%�ٴν����Ų���%%%%%%%%%%%%%%%%%%%%%%%
                                           
                            Bridge_point_1 = State_collision;
                            %�ŵ�һ��֧��Ϊ��Ӧ����ײ��
                            Bridge_vect = 2*rand(Dim,1) - 1;                            
                            %�Ų��Է���
                            if rand < P_bridge %һ�������£�����PCAƫ���Ų��Է���            
                                if Num_points_free >= Dim+1      %PCAƫ���Ų��Է���
                                    [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight_1*Points_free)');
                                    Num_PCA_compute_2 = Num_PCA_compute_2 + 1;                
                                    Bridge_vect = BridgeProject(Bridge_vect, Coeff_free, Latent_free, Dim);
                                end
                            end

                            Bridge_length = Radius + (Radius/3)*randn;          
                            %�Ų��Գ���
                            Bridge_point_2 = State_collision + Bridge_vect*Bridge_length/Distance(Bridge_vect, zeros(Dim, 1), Weight_1, 1);
                            %�ŵڶ���֧��
                            Middle_poMiddint = (Bridge_point_1 + Bridge_point_2) /2;
                            %���е�

                            Num_collision_test = Num_collision_test + 1;
                            if IsCollision(Bridge_point_2, Boundary, RobotData, ObstacleData, Robot_parameter) ~= 0     %����һ���㷢����ײ
                                Num_collision_test = Num_collision_test + 1;
                                if IsCollision(Middle_point, Boundary, RobotData, ObstacleData, Robot_parameter) == 0   %���е�����ײ
                                    [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Middle_point, Num_entrance_points, Weight_1, Error_goal);                
                                    if IsSucceedAdd == 1
                                        Entrance_flag(:, Num_entrance_points) = 0;               
                                        %��¼���ܵ�ͨ�����
                                        Flag_new_entrance = 1;  %��ʾ�Ų����ҵ����
                                        break
                                    end               
                                end            
                            end   
                        end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%�ڶ����Ų��Խ���%%%%%%%%%%%%%%%%%%%%%%
                    end
                    %%%%%%%%%%%!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    
                else

                    if Judge_node_from_in_ellipsoid == 0
                        % Node_from������խͨ����ڴ�                    

                        %%%%%%%�����ڽڵ�%%%%%%%%%
                        [~, Num_temp] = size(Points_free_in_ellipsoid);
                        Points_center = sum(Points_free_in_ellipsoid, 2)/Num_temp;

                        [Entrance_set, Num_entrance_points, IsSucceedAdd] = AddPoint(Entrance_set, Points_center, Num_entrance_points, Weight_1, Error_goal);                
                        if IsSucceedAdd == 1
                            Entrance_flag(:, Num_entrance_points) = 0;               
                            %��¼���ܵ�ͨ�����                   
                            Flag_new_entrance = 1;  %��ʾ�ҵ����                      
                        end                   

                        %%%%%%%%%%%%%%%%%%%%%%%%
    %                   	Node_parent = Tree.Node(:, Tree.Parent(From_index));
    %                     if (Weight*(Node_parent - Node_from))' * (Weight*(Node_rand - Node_from)) < 0
                            [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight_1*Points_free)');
                            Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
                            Num_node_to = 1;

                            State_temp_goal = 1*(State_rand - State_from) + State_from;    %��Ŀ���ӳ���ʹ������ͨ���о�������չ
                            State_to = NodeProject(State_from, State_temp_goal, Coeff_free, Latent_free);
                            %������ڵ���������չ�����Ͻ���ͶӰ(���ɿռ����չ����)     
    %                     end                          
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                        Type_point = 2;  %��ʾPCA�������ҵ����

                    else
                        %Node_from��խͨ���ڣ������ڷ�͹�ϰ��ﴦ��
                        %ȷ����չ����
    %                   	Node_parent = Tree.Node(:, Tree.Parent(From_index));
    %                     if (Weight*(Node_parent - Node_from))' * (Weight*(Node_rand - Node_from))  < 0
                            [Coeff_free, Score_free, Latent_free, Tsquared_free, Explained_free, Mu_free] = pca((Weight_1*Points_free)');
                            Num_PCA_compute_2 = Num_PCA_compute_2 + 1;
                            Num_node_to = 1;

                            State_temp_goal = 1*(State_rand - State_from) + State_from;    %��Ŀ���ӳ���ʹ������ͨ���о�������չ
                            State_to = NodeProject(State_from, State_temp_goal, Coeff_free, Latent_free);
                            Type_point = 3;  %��ʾPCA��������ͨ����
    %                     end
                    end
                end
            end

        end

        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%���������������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%�ȼ�¼��ʼ�ڵ㣬���ڶ����չ
        STATE_FROM = State_from;
        FROM_INDEX = From_index;

        %%%%���1������ҵ�����ڴ��Ľڵ㣬�ȶ���ڽڵ������չ
        if Flag_new_entrance == 1
            State_from = STATE_FROM;
            From_index = FROM_INDEX;

            Num_extend = 1;
            Num_extand_max = +3;  %��ʾ�����չ���� 5
            while true
                [State, End, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                        Steer(Tree, From_index, Entrance_set(:, Num_entrance_points), Step, Boundary, RobotData, ObstacleData, Robot_parameter, Num_collision_test, Weight_1, Error_goal);
                %Type_steer = 1, ǰ��һ��
                %Type_steer = 2�����������
                %Type_steer = 3, ������ײ

                if Judge == 0
                    Tree.State(:, Node_added_index) = State;
                    Tree.End (:, Node_added_index) = End;
                    Tree.Parent(Node_added_index) = From_index;
                    Tree.Children_num(From_index) = Tree.Children_num(From_index) + 1;
                    Tree.Children(Tree.Children_num(From_index), From_index) = Node_added_index;  
                    Tree.Type(:, Node_added_index) = 0;
                	Tree.ADD_Radius(Node_added_index) = +inf;

                    Tree.Children_num(Node_added_index) = 0;
                    Tree.Children(1, Node_added_index) = 0; %��������ӽڵ������ʼ��

                    Node_added_index = Node_added_index + 1;

                    Dist_temp = Distance(End, End_goal, Weight_2, 2);

                    if Dist_temp < Error_goal*10
                        Success_num = Success_num + 1;
                        Success(1, Success_num) = Node_added_index - 1;
                        Success(2, Success_num) = Dist_temp;

                        disp(['��' num2str(Success_num) '�γɹ��ҵ���' num2str(Success_num) '·������ ' num2str(toc) ' s']);
                        break;
                    end

                else
                    [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight_1, Error_goal);

                    %�����ײ���;���Ŀ����ڹ��ͣ���Ŀ����ڹ��ͱ��Ϊһ̽��״̬
                    if Distance(Entrance_set(:, Num_entrance_points), State_collision, Weight_1, 1) < Error_goal
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
            State_from = STATE_FROM;
            From_index = FROM_INDEX;

            Num_extend = 1;

            if  Type_point == 1 
                %��ʾ���ű߽���չ�����߸ս���ͨ��
                %�����ֻ��չһ���Ĳ��������ߵ���Ŀ���
                Num_extand_max = +1;% 20
                Node_type = 2;      %��ʾ���е������

            else
                if Type_point == 2
                    %��ʾ��խͨ���ڣ��򾡿�����չ��ֱ��������ײ
                    Num_extand_max = +3;  %+inf/ 5
                    Node_type = 3;      %��ʾ���е������
                else    % ��Type_point ==3
                    %��ʾ��խͨ���ڣ��򾡿�����չ��ֱ��������ײ
                    Num_extand_max = +3; %+inf/ 5
                    Node_type = 3;      %��ʾ���е������
                end
            end

            while true
                [State, End, Cost, Judge, Type_steer, State_collision, Num_collision_test] = ...
                        Steer(Tree, From_index, State_to, Step, Boundary, RobotData, ObstacleData, Robot_parameter, Num_collision_test, Weight_1, Error_goal);
                %Type_steer = 1, ǰ��һ��
                %Type_steer = 2�����������
                %Type_steer = 3, ������ײ

                if Judge == 0
                    Tree.State(:, Node_added_index) = State;
                    Tree.End (:, Node_added_index) = End;
                    Tree.Cost(:, Node_added_index) = Cost;
                    Tree.Parent(Node_added_index) = From_index;
                    Tree.Children_num(From_index) = Tree.Children_num(From_index) + 1;
                    Tree.Children(Tree.Children_num(From_index), From_index) = Node_added_index;  
                    Tree.Type(:, Node_added_index) = Node_type;
                 	Tree.ADD_Radius(Node_added_index) = +inf;   

                    Tree.Children_num(Node_added_index) = 0;
                    Tree.Children(1, Node_added_index) = 0; %��������ӽڵ������ʼ��

                    Node_added_index = Node_added_index + 1;

                    Dist_temp = Distance(End, End_goal, Weight_2, 2);

                    if Dist_temp < Error_goal*10
                        Success_num = Success_num + 1;
                        Success(1, Success_num) = Node_added_index - 1;
                        Success(2, Success_num) = Dist_temp;

                        disp(['��' num2str(Success_num) '�γɹ��ҵ���' num2str(Success_num) '·������ ' num2str(toc) ' s']);
                        break;
                    end

                else
                    [Collision_boundary_set, Num_collision_points, IsSuccessAdd] = AddPoint(Collision_boundary_set, State_collision, Num_collision_points, Weight_1, Error_goal);               
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
        
%         %% �����ж��Ƿ���Ҫ��������
%         if  Judge ~= 0 && Num_extend ~= 1 && Num_continue < 5%�����ͨ���У����ٷ�����һ����չ���Ƿ�������ײ�������������չ        	
%         	IsContinue = true;
%             Num_continue = Num_continue + 1;
%         else
%             IsContinue = false;
%         end
%         
%     end
    
    
    
end

Total_node = Node_added_index - 1;          %�ҵ����ܽڵ���
Result = [Iter_num, Node_added_index-1, Num_collision_test, Num_collision_points, Num_PCA_compute_1, Num_PCA_compute_2, toc];
disp(['�ܹ������� ' num2str(Iter_num) ' �ε��������� ' num2str(Node_added_index-1) ' ���ڵ㣬���� ' num2str(toc) ' s']);
disp(['�ܹ������� ' num2str(Num_collision_test) ' ����ײ��⣬���� ' num2str(Num_collision_points) ' ����ײ��']);
disp(['�ܹ������� ' num2str(Num_PCA_compute_1) ' �ε�һ��PCA����,�� ' num2str(Num_PCA_compute_2) ' �ε�һ��PCA����']);
disp(['�ܹ������� ' num2str(toc) ' s']);

% %������
Solution = TreePlot(Tree, Total_node, Success_num, Success, State_goal, End_goal, ObstacleData, Weight_2);
Movie = PlayMovie(Tree, Solution, RobotData, ObstacleData, Robot_parameter);
% end

