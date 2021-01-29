function Ae = Transpoort_Ae(RobotState)
    %从末端坐标系到第三连杆坐标系
    global a1 a2 a3 ;           %当前连杆质心到下一个关节的距离
    global b0 b1 b2 b3 ;        %前一个关节到当前连杆质心的距离
    
    x0 = RobotState(1);
    y0 = RobotState(2);
    q0 = RobotState(3);
    q1 = RobotState(4);
    q2 = RobotState(5);
    q3 = RobotState(6);    

    Ae = [ 1, 0, 0, a3 + b3 
           0, 1, 0,       0 
           0, 0, 1,       0 
           0, 0, 0,       1];

end