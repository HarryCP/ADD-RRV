function T3 = Transport_T3(RobotState)
%从第三连杆坐标系到惯性坐标系
    global a1 a2 a3 ;           %当前连杆质心到下一个关节的距离
    global b0 b1 b2 b3 ;        %前一个关节到当前连杆质心的距离

    x0 = RobotState(1);
    y0 = RobotState(2);
    q0 = RobotState(3);
    q1 = RobotState(4);
    q2 = RobotState(5);
    q3 = RobotState(6);    
    %机器人部分状态

   T3 = [ cos(q0 + q1 + q2 + q3), -sin(q0 + q1 + q2 + q3), 0, x0 + cos(q0 + q1)*(a1 + b1) + b0*cos(q0) + cos(q0 + q1 + q2)*(a2 + b2) 
           sin(q0 + q1 + q2 + q3),  cos(q0 + q1 + q2 + q3), 0, y0 + sin(q0 + q1)*(a1 + b1) + b0*sin(q0) + sin(q0 + q1 + q2)*(a2 + b2) 
                                0,                       0, 1,                                                                      0 
                                0,                       0, 0,                                                                      1];
   
end