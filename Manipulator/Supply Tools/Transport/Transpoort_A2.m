function A2 = Transpoort_A2(RobotState)
    %从第二连杆坐标系到第一连杆坐标系
    global a1 a2 a3 ;           %当前连杆质心到下一个关节的距离
    global b0 b1 b2 b3 ;        %前一个关节到当前连杆质心的距离
    
    x0 = RobotState(1);
    y0 = RobotState(2);
    q0 = RobotState(3);
    q1 = RobotState(4);
    q2 = RobotState(5);
    q3 = RobotState(6);    

    A2 = [ cos(q2), -sin(q2), 0, a1 + b1 
           sin(q2),  cos(q2), 0,       0 
                 0,        0, 1,       0 
                 0,        0, 0,       1];

end