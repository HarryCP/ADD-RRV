function Judge = FirstCollisionCheck(RobotState)
    %��ץ����Ϊ����ϵΪ�ο�����ϵ
    % 1��ʾ������ײ
    % 0��ʾδ������ײ
    global RobotData TargetData;
    global a1 a2 a3 ;          
    global b0 b1 b2 b3 ;
    
    Judge = 0;
    
    %��������
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %�����˲���
    T0 = Transport_T0(RobotState);
    T1 = Transport_T1(RobotState);
    T2 = Transport_T2(RobotState);
    T3 = Transport_T3(RobotState);
 
    Robot_Base_I = T0 * RobotData.Base_C;
    Robot_Panel1_I = T0 * RobotData.Panel1_C;
    Robot_Panel2_I = T0 * RobotData.Panel2_C;
    Robot_Link1_I = T1 * RobotData.Link1_C;
    Robot_Link2_I = T2 * RobotData.Link2_C;
    Robot_Link3_I = T3 * RobotData.Link3_C;    
    %���Կռ�����
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Ŀ�����
    Target_base_width = 2;     %�������
    Target_base_length = 2;    %��������
    Target_panel_width = 1;    %ƽ����
    Target_panel_length = 3;   %ƽ�峤��
                
    Transport = [ 1, 0, 0, Target_base_width/2 
                  0, 1, 0, 0 
                  0, 0, 1, 0 
                  0, 0, 0, 1];
    
    Target_Base_I = Transport * TargetData.Base_C;
    Target_Panel1_I = Transport * TargetData.Panel1_C;
    Target_Panel2_I = Transport * TargetData.Panel2_C;
    
    %�Ի���������ײ���м��
    if GJK(Robot_Link2_I(1:2,:), Robot_Base_I(1:2,:)) || GJK(Robot_Link2_I(1:2,:), Robot_Panel1_I(1:2,:)) || GJK(Robot_Link2_I(1:2,:), Robot_Panel2_I(1:2,:)) || ...
            GJK(Robot_Link3_I(1:2,:), Robot_Link1_I(1:2,:)) || GJK(Robot_Link3_I(1:2,:), Robot_Base_I(1:2,:)) || GJK(Robot_Link3_I(1:2,:), Robot_Panel1_I(1:2,:)) || ...
            GJK(Robot_Link3_I(1:2,:), Robot_Panel2_I(1:2,:))
        
        Judge = 1;
        return;
    end
    
    if  GJK(Robot_Link3_I(1:2,:), Target_Panel1_I(1:2,:)) || GJK(Robot_Link3_I(1:2,:), Target_Panel2_I(1:2,:)) || GJK(Robot_Link3_I(1:2,:), Target_Base_I(1:2,:)) || ...
            GJK(Robot_Panel1_I(1:2,:), Target_Panel1_I(1:2,:)) || GJK(Robot_Panel1_I(1:2,:), Target_Panel2_I(1:2,:)) || GJK(Robot_Panel1_I(1:2,:), Target_Base_I(1:2,:)) || ...
            GJK(Robot_Panel2_I(1:2,:), Target_Panel1_I(1:2,:)) || GJK(Robot_Panel2_I(1:2,:), Target_Panel2_I(1:2,:)) || GJK(Robot_Panel2_I(1:2,:), Target_Base_I(1:2,:)) || ...
            GJK(Robot_Link2_I(1:2,:), Target_Panel1_I(1:2,:)) || GJK(Robot_Link2_I(1:2,:), Target_Panel2_I(1:2,:)) || GJK(Robot_Link2_I(1:2,:), Target_Base_I(1:2,:)) || ... 
            GJK(Robot_Link1_I(1:2,:), Target_Panel1_I(1:2,:)) || GJK(Robot_Link1_I(1:2,:), Target_Panel2_I(1:2,:)) || GJK(Robot_Link1_I(1:2,:), Target_Base_I(1:2,:)) || ... 
            GJK(Robot_Base_I(1:2,:), Target_Panel1_I(1:2,:)) || GJK(Robot_Base_I(1:2,:), Target_Panel2_I(1:2,:)) || GJK(Robot_Base_I(1:2,:), Target_Base_I(1:2,:))
        
        Judge = 1;
        return;
    
    end   
  
end



