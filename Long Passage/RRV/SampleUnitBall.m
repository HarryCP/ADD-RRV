%�ڵ�λ���ھ��Ȳ���
function Point = SampleUnitBall(Dimention, Number)
    %Dimention��ʾ��λ���ά��
    %Number��ʾ��������
    Point = zeros(Dimention, Number);
    Sample = zeros(Dimention, 1);
    R = 1;          %��ʾʣ��ɲ����뾶����
    r_used = 0;     %��ʹ�õĲ����뾶���ȵ�ƽ�� 
        
    Dim = 1;
    
    while Dim <= Dimention
        Sample(Dim) = 2*R*rand() - R;
        r_used = r_used + Sample(Dim)^2;
        R = (1 - r_used)^0.5;
        Dim = Dim + 1;
    end
    
    Num = 1;
    while Num <= Number
        Dim = 1;    
        while Dim <= Dimention
            r_used =  sum(Sample.^2) - Sample(Dim)^2;
            R = (1 - r_used)^0.5;      
            Sample(Dim) = 2*R*rand() - R;
            Dim = Dim + 1;
        end
        
        Point(:, Num) = Sample;
        Num = Num + 1;
    end
    
    
    
    
    
end