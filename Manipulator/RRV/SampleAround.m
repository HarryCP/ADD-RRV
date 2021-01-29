%在某点附近进行随机采样    
function Points_sample = SampleAround(Node_from, Num_near_node, Radius, Boundary, Seed_num, Seed_unit_ball, Weight)
    %围绕Node_from在半径Radius随机采样Num_near_node个点
    %采样不能超过规划范围
    
    %% 在矩形区域内采样
%     Dim = length(Node_from);    %样本维度
%     Boundary_modify = zeros(Dim, 2);
    
%     for kk = 1:Dim
%         Boundary_modify(kk, 1) = max(Boundary(kk, 1), Node_from(kk, :) - Radius);  
%         Boundary_modify(kk, 2) = min(Boundary(kk, 2), Node_from(kk, :) + Radius);        
%     end
    
%     Points_sample = (Boundary_modify(:, 2) - Boundary_modify(:, 1)) .* rand(Dim, Num_near_node) + Boundary_modify(:, 1);

    
    %% 在单位球内采样
    Points_sample = inv(Weight) * (Radius * Seed_unit_ball(:, randi([1,Seed_num], 1, Num_near_node))) + Node_from;
     
end
    