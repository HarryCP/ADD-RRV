%��������̬�ǵ����
%����ֵΪ��ֵ
function Difference = AttitudeDifference(Atti1, Atti2)
	
    Difference = abs(Atti1-Atti2);
    
    Index = find( Difference > pi );
        
    while isempty(Index) ~= 1
        Difference(Index) = abs( Difference(Index) - 2*pi );
        Index = find( Difference > pi );
    end
    

end

