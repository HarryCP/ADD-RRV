function Judge = IsCollision(Robot_state, Boundary, RobotData, ObstacleData, Robot_parameter)
    Judge = 0;
    % 1��ʾ������ײ
    % 0��ʾδ������ײ
    L1 = Robot_parameter.L1;
    L2 = Robot_parameter.L2;
    L3 = Robot_parameter.L3;
    L4 = Robot_parameter.L4;
    L5 = Robot_parameter.L5;
    L6 = Robot_parameter.L6;

    W1 = Robot_parameter.W1;
    W2 = Robot_parameter.W2;
    W3 = Robot_parameter.W3;
    W4 = Robot_parameter.W4;
    W5 = Robot_parameter.W5;
    W6 = Robot_parameter.W6;        
           
    %% �ж�״̬�Ƿ��Խ��
    State_boundary =  [ Boundary.q1;
                        Boundary.q2; 
                        Boundary.q3; 
                        Boundary.q4; 
                        Boundary.q5; 
                        Boundary.q6];
                    
     Num = length(Robot_state);
     
     for i = 1:Num
        if  ( Robot_state(i) < State_boundary(i, 1) || Robot_state(i) > State_boundary(i, 2) )
            Judge = 1; %1!!!!!!!!!!!!!!!!!!!!!!
            return;
        end         
     end 
     
    %% ���������ײ���
    %return
    %���ȣ��ֽ������Ŀ��ֱ���һ����Բ�����������ж��Ƿ�����ײ
    %����Բ������ײ���ٽ�һ���жϾ�ȷģ���Ƿ�����ײ
    
    %��������
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %�����˲���
    
	L1 = Robot_parameter.L1;
    L2 = Robot_parameter.L2;
    L3 = Robot_parameter.L3;
    L4 = Robot_parameter.L4;
    L5 = Robot_parameter.L5;
    L6 = Robot_parameter.L6;

    W1 = Robot_parameter.W1;
    W2 = Robot_parameter.W2;
    W3 = Robot_parameter.W3;
    W4 = Robot_parameter.W4;
    W5 = Robot_parameter.W5;
    W6 = Robot_parameter.W6;     
    
	Trans_j1 = Trans([5, 5, 0]')*RotZ(Robot_state(1, :));    
    %��1���˹ؽ�����ڻ����任����
    Trans_c1 = Trans_j1*Trans([L1/2, 0, 0]');
    %��1������������ڻ����任����
    
    Trans_j2 = Trans_j1*Trans([L1, 0, 0]')*RotZ(Robot_state(2, :));    
    %��2���˹ؽ�����ڻ����任����
    Trans_c2 = Trans_j2*Trans([L2/2, 0, 0]');
    %��2������������ڻ����任����
    
	Trans_j3 = Trans_j2*Trans([L2, 0, 0]')*RotZ(Robot_state(3, :));    
    %��3���˹ؽ�����ڻ����任����
    Trans_c3 = Trans_j3*Trans([L3/2, 0, 0]');
    %��3������������ڻ����任����
    
    Trans_j4 = Trans_j3*Trans([L3, 0, 0]')*RotZ(Robot_state(4, :));    
    %��4���˹ؽ�����ڻ����任����
    Trans_c4 = Trans_j4*Trans([L4/2, 0, 0]');
    %��4������������ڻ����任����
    
    Trans_j5 = Trans_j4*Trans([L4, 0, 0]')*RotZ(Robot_state(5, :));    
    %��5���˹ؽ�����ڻ����任����
    Trans_c5 = Trans_j5*Trans([L5/2, 0, 0]');
    %��5������������ڻ����任����    
    
	Trans_j6 = Trans_j5*Trans([L5, 0, 0]')*RotZ(Robot_state(6, :));    
    %��6���˹ؽ�����ڻ����任����
    Trans_c6 = Trans_j6*Trans([L6/2, 0, 0]');
    %��6������������ڻ����任����
    
	Robot_Link1_I = Trans_c1 * RobotData.Link1_C;
    Robot_Link2_I = Trans_c2 * RobotData.Link2_C;    
    Robot_Link3_I = Trans_c3 * RobotData.Link3_C;
    Robot_Link4_I = Trans_c4 * RobotData.Link4_C;
    Robot_Link5_I = Trans_c5 * RobotData.Link5_C;
    Robot_Link6_I = Trans_c6 * RobotData.Link6_C;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %�Ի���������ײ���м��
    if GJK(Robot_Link6_I(1:2,:), 1, Robot_Link4_I(1:2,:), 1) ||  GJK(Robot_Link6_I(1:2,:), 1, Robot_Link3_I(1:2,:), 1) ||  GJK(Robot_Link6_I(1:2,:), 1, Robot_Link2_I(1:2,:), 1) ...
            ||  GJK(Robot_Link6_I(1:2,:), 1, Robot_Link1_I(1:2,:), 1) ||  GJK(Robot_Link5_I(1:2,:), 1, Robot_Link3_I(1:2,:), 1) ||  GJK(Robot_Link5_I(1:2,:), 1, Robot_Link2_I(1:2,:), 1) ...
            ||  GJK(Robot_Link5_I(1:2,:), 1, Robot_Link1_I(1:2,:), 1) ||  GJK(Robot_Link4_I(1:2,:), 1, Robot_Link2_I(1:2,:), 1) ||  GJK(Robot_Link4_I(1:2,:), 1, Robot_Link1_I(1:2,:), 1) ...
            ||  GJK(Robot_Link3_I(1:2,:), 1, Robot_Link1_I(1:2,:), 1)
        Judge = 1;
        return;
    end
    %%%%%%%%%%%%%%%%%%%%%%%    
    %%%%�Ծ�ֹ�ϰ��������ײ���
    %�ȶ�1�ž�ֹ�ϰ�����м��
	if GJK(Robot_Link6_I(1:2,:), 1, ObstacleData.NO1(1:2, :), 1) || GJK(Robot_Link5_I(1:2,:), 1, ObstacleData.NO1(1:2, :), 1) || GJK(Robot_Link4_I(1:2,:), 1, ObstacleData.NO1(1:2, :), 1) || ...
    	GJK(Robot_Link3_I(1:2,:), 1, ObstacleData.NO1(1:2, :), 1) || GJK(Robot_Link2_I(1:2,:), 1, ObstacleData.NO1(1:2, :), 1) || GJK(Robot_Link1_I(1:2,:), 1, ObstacleData.NO1(1:2, :), 1)
    	Judge = 1;
        return;
	end
    
    %��2�ž�ֹ�ϰ��������ײ���
	if GJK(Robot_Link6_I(1:2,:), 1, ObstacleData.NO2(1:2, :), 1) || GJK(Robot_Link5_I(1:2,:), 1, ObstacleData.NO2(1:2, :), 1) || GJK(Robot_Link4_I(1:2,:), 1, ObstacleData.NO2(1:2, :), 1) || ...
    	GJK(Robot_Link3_I(1:2,:), 1, ObstacleData.NO2(1:2, :), 1) || GJK(Robot_Link2_I(1:2,:), 1, ObstacleData.NO2(1:2, :), 1) || GJK(Robot_Link1_I(1:2,:), 1, ObstacleData.NO2(1:2, :), 1)
    	Judge = 1;
        return;
	end
    
    %��3�ž�ֹ�ϰ��������ײ���
    if GJK(Robot_Link6_I(1:2,:), 1, ObstacleData.NO3(1:2, :), 1) || GJK(Robot_Link5_I(1:2,:), 1, ObstacleData.NO3(1:2, :), 1) || GJK(Robot_Link4_I(1:2,:), 1, ObstacleData.NO3(1:2, :), 1) || ...
    	GJK(Robot_Link3_I(1:2,:), 1, ObstacleData.NO3(1:2, :), 1) || GJK(Robot_Link2_I(1:2,:), 1, ObstacleData.NO3(1:2, :), 1) || GJK(Robot_Link1_I(1:2,:), 1, ObstacleData.NO3(1:2, :), 1)
    	Judge = 1;
        return;
    end
    
%     %��4�ž�ֹ�ϰ��������ײ���
% 	if GJK(Robot_Link6_I(1:2,:), 1, ObstacleData.NO4(1:2, :), 1) || GJK(Robot_Link5_I(1:2,:), 1, ObstacleData.NO4(1:2, :), 1) || GJK(Robot_Link4_I(1:2,:), 1, ObstacleData.NO4(1:2, :), 1) || ...
%     	GJK(Robot_Link3_I(1:2,:), 1, ObstacleData.NO4(1:2, :), 1) || GJK(Robot_Link2_I(1:2,:), 1, ObstacleData.NO4(1:2, :), 1) || GJK(Robot_Link1_I(1:2,:), 1, ObstacleData.NO4(1:2, :), 1)
%     	Judge = 3;
%         return;
% 	end
end