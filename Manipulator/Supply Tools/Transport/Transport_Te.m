function Te = Transport_Te(RobotState, Robot_parameter)
    %��ĩ�˹�������ϵ����������ϵ
    a1 = Robot_parameter.a1;
    a2 = Robot_parameter.a2;
    a3 = Robot_parameter.a3;
    b0 = Robot_parameter.b0;
    b1 = Robot_parameter.b1;
    b2 = Robot_parameter.b2;
    b3 = Robot_parameter.b3;

    x0 = RobotState(1);
    y0 = RobotState(2);
    q0 = RobotState(3);
    q1 = RobotState(4);
    q2 = RobotState(5);
    q3 = RobotState(6);    
    %�����˲���״̬

  Te = [ cos(q0 + q1 + q2 + q3), -sin(q0 + q1 + q2 + q3), 0, x0 + a3*cos(q0 + q1 + q2 + q3) + b3*cos(q0 + q1 + q2 + q3) + a1*cos(q0 + q1) + b1*cos(q0 + q1) + b0*cos(q0) + a2*cos(q0 + q1 + q2) + b2*cos(q0 + q1 + q2) 
           sin(q0 + q1 + q2 + q3),  cos(q0 + q1 + q2 + q3), 0, y0 + a3*sin(q0 + q1 + q2 + q3) + b3*sin(q0 + q1 + q2 + q3) + a1*sin(q0 + q1) + b1*sin(q0 + q1) + b0*sin(q0) + a2*sin(q0 + q1 + q2) + b2*sin(q0 + q1 + q2) 
                                0,                       0, 1,                                                                                                                                                         0 
                                0,                       0, 0,                                                                                                                                                         1];
 

end