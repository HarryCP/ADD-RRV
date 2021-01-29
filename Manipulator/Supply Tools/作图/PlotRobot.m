function PlotRobot(Robot_state, Robot_parameter, RobotData)
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
    %第1连杆关节相对于基座变换矩阵
    Trans_c1 = Trans_j1*Trans([L1/2, 0, 0]');
    %第1连杆质心相对于基座变换矩阵
    
    Trans_j2 = Trans_j1*Trans([L1, 0, 0]')*RotZ(Robot_state(2, :));    
    %第2连杆关节相对于基座变换矩阵
    Trans_c2 = Trans_j2*Trans([L2/2, 0, 0]');
    %第2连杆质心相对于基座变换矩阵
    
	Trans_j3 = Trans_j2*Trans([L2, 0, 0]')*RotZ(Robot_state(3, :));    
    %第3连杆关节相对于基座变换矩阵
    Trans_c3 = Trans_j3*Trans([L3/2, 0, 0]');
    %第3连杆质心相对于基座变换矩阵
    
    Trans_j4 = Trans_j3*Trans([L3, 0, 0]')*RotZ(Robot_state(4, :));    
    %第4连杆关节相对于基座变换矩阵
    Trans_c4 = Trans_j4*Trans([L4/2, 0, 0]');
    %第4连杆质心相对于基座变换矩阵
    
    Trans_j5 = Trans_j4*Trans([L4, 0, 0]')*RotZ(Robot_state(5, :));    
    %第5连杆关节相对于基座变换矩阵
    Trans_c5 = Trans_j5*Trans([L5/2, 0, 0]');
    %第5连杆质心相对于基座变换矩阵    
    
	Trans_j6 = Trans_j5*Trans([L5, 0, 0]')*RotZ(Robot_state(6, :));    
    %第6连杆关节相对于基座变换矩阵
    Trans_c6 = Trans_j6*Trans([L6/2, 0, 0]');
    %第6连杆质心相对于基座变换矩阵
    

    %% 绘制机器人                
    %第1连杆
    Joint1_C = [0, 0, 0, 1]';
    
    Joint1_I = Trans_j1 * Joint1_C;
    Robot_Link1_I = Trans_c1 * RobotData.Link1_C;
    
    Link_X1 = Robot_Link1_I(1, :);
    Link_Y1 = Robot_Link1_I(2, :);
    
    plot(Link_X1, Link_Y1, 'b', 'LineWidth', 1);
    fill(Link_X1, Link_Y1, 'b');
    PlotCircle(Joint1_I, 'r', 1)
    
    %第2连杆
    Joint2_C = [0, 0, 0, 1]';
                
    Joint2_I = Trans_j2 * Joint2_C;
    Robot_Link2_I = Trans_c2 * RobotData.Link2_C;
    
    Link_X2 = Robot_Link2_I(1, :);
    Link_Y2 = Robot_Link2_I(2, :);
    
    plot(Link_X2, Link_Y2, 'b', 'LineWidth', 1);
    fill(Link_X2, Link_Y2, 'b');
    PlotCircle(Joint2_I, 'r', 1)
    
     %第3连杆
    Joint3_C = [0, 0, 0, 1]';
                
    Joint3_I = Trans_j3 * Joint3_C;
    Robot_Link3_I = Trans_c3 * RobotData.Link3_C;
    
    Link_X3 = Robot_Link3_I(1, :);
    Link_Y3 = Robot_Link3_I(2, :);
    
    plot(Link_X3, Link_Y3, 'b', 'LineWidth', 1);
    fill(Link_X3, Link_Y3, 'b');
    PlotCircle(Joint3_I, 'r', 1)
    
	%第4连杆
    Joint4_C = [0, 0, 0, 1]';
                
    Joint4_I = Trans_j4 * Joint4_C;
    Robot_Link4_I = Trans_c4 * RobotData.Link4_C;
    
    Link_X4 = Robot_Link4_I(1, :);
    Link_Y4 = Robot_Link4_I(2, :);
    
    plot(Link_X4, Link_Y4, 'b', 'LineWidth', 1);
    fill(Link_X4, Link_Y4, 'b');
    PlotCircle(Joint4_I, 'r', 1)
    
    %第5连杆
    Joint5_C = [0, 0, 0, 1]';
                
    Joint5_I = Trans_j5 * Joint5_C;
    Robot_Link5_I = Trans_c5 * RobotData.Link5_C;
    
    Link_X5 = Robot_Link5_I(1, :);
    Link_Y5 = Robot_Link5_I(2, :);
    
    plot(Link_X5, Link_Y5, 'b', 'LineWidth', 1);
    fill(Link_X5, Link_Y5, 'b');
    PlotCircle(Joint5_I, 'r', 1)
    
    %第6连杆
    Joint6_C = [0, 0, 0, 1]';
                
    Joint6_I = Trans_j6 * Joint6_C;
    Robot_Link6_I = Trans_c6 * RobotData.Link6_C;
    
    Link_X6 = Robot_Link6_I(1, :);
    Link_Y6 = Robot_Link6_I(2, :);
    
    plot(Link_X6, Link_Y6, 'b', 'LineWidth', 1);
    fill(Link_X6, Link_Y6, 'b');
    PlotCircle(Joint6_I, 'r', 1)
end