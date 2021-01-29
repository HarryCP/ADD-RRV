%计算末端位姿
function End = ForwardKine(Robot_state, Robot_parameter)
    End = zeros(3, 1);
    
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
    
    End_temp = Trans_j6 * [L6, 0, 0, 1]';
    End(1:2, 1) = End_temp(1:2, 1);
    End(3, :) = GetAttiFromTrans(Trans_j6);
    



end