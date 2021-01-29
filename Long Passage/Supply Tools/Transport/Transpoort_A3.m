function A3 = Transpoort_A3(RobotState)
    %�ӵ�����������ϵ���ڶ���������ϵ
    global a1 a2 a3 ;           %��ǰ�������ĵ���һ���ؽڵľ���
    global b0 b1 b2 b3 ;        %ǰһ���ؽڵ���ǰ�������ĵľ���
    
    x0 = RobotState(1);
    y0 = RobotState(2);
    q0 = RobotState(3);
    q1 = RobotState(4);
    q2 = RobotState(5);
    q3 = RobotState(6);    

    A3 = [ cos(q3), -sin(q3), 0, a2 + b2 
           sin(q3),  cos(q3), 0,       0 
                 0,        0, 1,       0 
                 0,        0, 0,       1];

end