%PCAƫִ�Ų��Է���
function Bridge_vect_new = BridgeProject(Bridge_vect, Coeff, Latent, Dim)
	% CoeffΪЭ�������������������������������任����
    % LatentΪ������������Ӧ������ֵ���Ӵ�С����
    
    Node_temp = (Bridge_vect'*Coeff)';
    %����任����Node_fromΪԭ�㣬�任�������ϵ��
        
    for kk = 1:Dim
       Node_temp(kk, :) = Bridge_vect(kk, :) * Latent(Dim)/ Latent(kk); 
        %��������ֵ�Ե����ͶӰ
    end
    
	Bridge_vect_new = (Node_temp' / Coeff)';
    %��ͶӰ��ĵ㣬�任��ԭ����ϵ��
    

end
