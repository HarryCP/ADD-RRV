%Ëæ»úÈöµã
function State_rand= RandomSample( P_sample, Boundary, State_goal)
    if rand(1) < P_sample
        State_rand = State_goal;

    else
        BOUNDARY = [Boundary.q1
                    Boundary.q2
                    Boundary.q3
                    Boundary.q4
                    Boundary.q5
                    Boundary.q6];
                
        State_rand = (BOUNDARY(:, 2) - BOUNDARY(:, 1)) .* rand(6, 1) + BOUNDARY(:, 1);

    end
    
end