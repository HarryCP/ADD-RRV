%���ڼ�������״̬�����Ķ�������
function Dist = Distance(From, To, Weight, Flag)
    
    %�ü�Ȩ���������һ��
    Diff = (From - To);
    if Flag == 2    %��ʾ����ά����̬
        Diff(3, :) = AttitudeDifference(From(3, :), To(3, :));
    end
    
    Dist =  sum((Weight*Diff) .^ 2) .^ 0.5;
   
end