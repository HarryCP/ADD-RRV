function Judge = FirstCollisionCheck(RobotState)
    %以抓捕点为坐标系为参考坐标系
    % 1表示发生碰撞
    % 0表示未发生碰撞
    global RobotData TargetData;
    global a1 a2 a3 ;          
    global b0 b1 b2 b3 ;
    
    Judge = 0;
    
    %参数设置
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %机器人参数
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
    %惯性空间坐标
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %目标参数
    Target_base_width = 2;     %基座宽度
    Target_base_length = 2;    %基座长度
    Target_panel_width = 1;    %平板宽度
    Target_panel_length = 3;   %平板长度
                
    Transport = [ 1, 0, 0, Target_base_width/2 
                  0, 1, 0, 0 
                  0, 0, 1, 0 
                  0, 0, 0, 1];
    
    Target_Base_I = Transport * TargetData.Base_C;
    Target_Panel1_I = Transport * TargetData.Panel1_C;
    Target_Panel2_I = Transport * TargetData.Panel2_C;
    
    %对机器人自碰撞进行检测
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



