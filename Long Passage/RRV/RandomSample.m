%Ëæ»úÈöµã
function Node_rand = RandomSample( P_sample, Boundary, State_goal)
    if rand(1) < P_sample
        Node_rand = State_goal;
    else
        Node_rand = (Boundary(:, 2) - Boundary(:, 1)) .* rand(3, 1) + Boundary(:, 1);
    end
    
end