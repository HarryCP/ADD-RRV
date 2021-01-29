%在单位球内均匀采样
function Point = SampleUnitBall(Dimention, Number)
    %Dimention表示单位球的维度
    %Number表示样本个数
    Point = zeros(Dimention, Number);
    Sample = zeros(Dimention, 1);
    R = 1;          %表示剩余可采样半径长度
    r_used = 0;     %已使用的采样半径长度的平方 
        
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