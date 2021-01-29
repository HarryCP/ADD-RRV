%PCA偏执桥测试方向
function Bridge_vect_new = BridgeProject(Bridge_vect, Coeff, Latent, Dim)
	% Coeff为协方差矩阵的特性向量矩阵（列向量），变换矩阵
    % Latent为各特征向量对应的特征值，从大到小排列
    
    Node_temp = (Bridge_vect'*Coeff)';
    %将点变换到以Node_from为原点，变换后的坐标系下
        
    for kk = 1:Dim
       Node_temp(kk, :) = Bridge_vect(kk, :) * Latent(Dim)/ Latent(kk); 
        %根据特征值对点进行投影
    end
    
	Bridge_vect_new = (Node_temp' / Coeff)';
    %将投影后的点，变换到原坐标系下
    

end
