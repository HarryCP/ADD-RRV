function A2 = Transpoort_A2(RobotState)
    %�ӵڶ���������ϵ����һ��������ϵ
    global a1 a2 a3 ;           %��ǰ�������ĵ���һ���ؽڵľ���
    global b0 b1 b2 b3 ;        %ǰһ���ؽڵ���ǰ�������ĵľ���
    
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