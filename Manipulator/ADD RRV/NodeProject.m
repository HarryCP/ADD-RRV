%������ڵ���������չ�����Ͻ���ͶӰ
function Node_to = NodeProject(Node_from, Node_rand, Coeff, Latent)    
    % CoeffΪЭ�������������������������������任����
    % LatentΪ������������Ӧ������ֵ���Ӵ�С����
    % �Ե���б任��Score = (X-X0) * Coeff.
    Dim = length(Latent);
    
    Node_temp = ((Node_rand - Node_from)'*Coeff)';
    %����任����Node_fromΪԭ�㣬�任�������ϵ��
    
    for kk = 1:Dim
       Node_temp(kk, :) = Node_temp(kk, :) * Latent(kk)/Latent(1); 
        %��������ֵ�Ե����ͶӰ
    end
    
    Node_to = (Node_temp' / Coeff)' + Node_from;
    %��ͶӰ��ĵ㣬�任��ԭ����ϵ��
    
    
    
    
end 