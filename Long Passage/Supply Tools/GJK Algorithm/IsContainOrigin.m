function [Judge , NewSimplix, NewDirection] = IsContainOrigin(OldSimplix, OldDirection, Flag)
    %�ж�ԭ���Ƿ��ڵ���������
	[m, n] = size(OldSimplix);
    
    NewSimplix = OldSimplix;
    NewDirection = zeros(m, 1);
    
    %m��ʾ�ռ�ά�ȣ�n��ʾ�����ζ�����Ŀ
    %Ŀǰֻ�����˶�ά�����
    PointA = OldSimplix(:, 1);
    PointB = OldSimplix(:, 2);
    PointC = OldSimplix(:, 3);
    Origin = zeros(m, 1);
    
    VectorAO = Origin - PointA;
    VectorAB = PointB - PointA;
    VectorAC = PointC - PointA;
    
    if Flag > 2    %��������������
        PerpAB = DoubleCross(VectorAC, VectorAB, VectorAB);     %AB�Ĵ��ߣ���ָ�򵥴����ⲿ
        PerpAC = DoubleCross(VectorAB, VectorAC, VectorAC);     %AC�Ĵ��ߣ���ָ�򵥴����ⲿ
        
        if PerpAB' * VectorAO > 0   %ԭ���ڵ�����AB�ߵ��ⲿ
            NewSimplix(:, 2) = OldSimplix(:, 1);
            NewSimplix(:, 3) = OldSimplix(:, 2);
            NewDirection = PerpAB;
            Judge = 0;
        else
            if PerpAC' * VectorAO > 0   %ԭ���ڵ�����AC�ߵ��ⲿ
                NewSimplix(:, 2) = OldSimplix(:, 1);
                NewSimplix(:, 3) = OldSimplix(:, 3);
                NewDirection = PerpAC;
                Judge = 0;
            else
                Judge = 1;           
            end 
        end
        
    else        %��������һ��ֱ��
        NewDirection = DoubleCross(VectorAB, VectorAO, VectorAB);
        NewSimplix(:, 2) = OldSimplix(:, 1);
       	NewSimplix(:, 3) = OldSimplix(:, 2);
        Judge = 0;
        
    end
    
    


end