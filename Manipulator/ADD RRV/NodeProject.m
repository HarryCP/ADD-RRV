%对随机节点在最优拓展方向上进行投影
function Node_to = NodeProject(Node_from, Node_rand, Coeff, Latent)    
    % Coeff为协方差矩阵的特性向量矩阵（列向量），变换矩阵
    % Latent为各特征向量对应的特征值，从大到小排列
    % 对点进行变换，Score = (X-X0) * Coeff.
    Dim = length(Latent);
    
    Node_temp = ((Node_rand - Node_from)'*Coeff)';
    %将点变换到以Node_from为原点，变换后的坐标系下
    
    for kk = 1:Dim
       Node_temp(kk, :) = Node_temp(kk, :) * Latent(kk)/Latent(1); 
        %根据特征值对点进行投影
    end
    
    Node_to = (Node_temp' / Coeff)' + Node_from;
    %将投影后的点，变换到原坐标系下
    
    
    
    
end 