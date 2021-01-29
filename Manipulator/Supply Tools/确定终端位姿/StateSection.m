function Option_robot_state = StateSection(Xe, Ye, Boundary, Robot_parameter, RobotData, ObstacleData)
    
    dq = deg2rad(8);    %·Ö±æÂÊ
    
    Option_robot_state = zeros(6, 1);
 
    i = 1;
    kk = 0;
    q1 =  Boundary.q1(1);
    while q1 <= Boundary.q1(2)
        q2 =  Boundary.q2(1);
        while q2 <= Boundary.q2(2)
            q3 =  Boundary.q3(1);
            while q3 <= Boundary.q3(2)
                theta_e = -pi;
                while theta_e <= pi
                    End_state = [Xe, Ye, theta_e]';
                    Base_state = GetIK(End_state, [q1, q2, q3]', Robot_parameter);
                    Robot_state = [ Base_state
                                    q1
                                    q2
                                    q3];
                           
                    if IsCollision(Robot_state, Boundary, RobotData, ObstacleData, Robot_parameter) == 0
                        Option_robot_state(:, i) = Robot_state;
                        i = i + 1
                    end
                    theta_e = theta_e + dq;
                    kk = kk +1;
                end               
                q3 = q3 + dq;               
            end
            q2 = q2 + dq;    
        end
        q1 = q1 + dq;       
    end
    
    save 'Option_robot_state.mat' Option_robot_state;
    
end

