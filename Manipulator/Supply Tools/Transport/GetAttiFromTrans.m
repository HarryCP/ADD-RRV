%�ڶ�άƽ���У�����ת����õ���̬��
function Attitude = GetAttiFromTrans(Trans)
    SA = Trans(2, 1);   %��̬�ǵ�cosֵ
    CA = Trans(1, 1);   %��̬�ǵ�sinֵ
    
    Attitude = asin(SA);
    
    if SA >= 0 && CA <= 0
       Attitude = pi - Attitude; 
    end
    
    if SA <= 0 && CA <= 0
       Attitude = -pi - Attitude; 
    end
    
end