function T0 = Transport_T0(RobotState)
    %�����Ļ�����������ϵ����������ϵ
    
    global a1 a2 a3 ;           %��ǰ�������ĵ���һ���ؽڵľ���
    global b0 b1 b2 b3 ;        %ǰһ���ؽڵ���ǰ�������ĵľ���

    x0 = RobotState(1);
    y0 = RobotState(2);
    q0 = RobotState(3);
    q1 = RobotState(4);
    q2 = RobotState(5);
    q3 = RobotState(6);    
    %�����˲���״̬
    
    
    T0 = [ cos(q0), -sin(q0), 0, x0 
           sin(q0),  cos(q0), 0, y0 
                 0,        0, 1,  0 
                 0,        0, 0,  1];

end