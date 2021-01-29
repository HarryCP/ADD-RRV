function T2 = Transport_T2(RobotState)
%�ӵڶ���������ϵ����������ϵ
    global a1 a2 a3 ;           %��ǰ�������ĵ���һ���ؽڵľ���
    global b0 b1 b2 b3 ;        %ǰһ���ؽڵ���ǰ�������ĵľ���

    x0 = RobotState(1);
    y0 = RobotState(2);
    q0 = RobotState(3);
    q1 = RobotState(4);
    q2 = RobotState(5);
    q3 = RobotState(6);    
    %�����˲���״̬

    T2 = [ cos(q0 + q1 + q2), -sin(q0 + q1 + q2), 0, x0 + cos(q0 + q1)*(a1 + b1) + b0*cos(q0) 
           sin(q0 + q1 + q2),  cos(q0 + q1 + q2), 0, y0 + sin(q0 + q1)*(a1 + b1) + b0*sin(q0) 
                           0,                  0, 1,                                        0 
                           0,                  0, 0,                                        1];
end