%���ڼ�������״̬�����Ķ�������
function Dist = Distance(From, To, Weight)
    %״̬������ά��λ�ú�һά����̬
    %�ü�Ȩ���������һ��
    Diff = (From - To);
    Diff(3, :) = AttitudeDifference(From(3, :), To(3, :));
    
    Dist =  sum((Weight*Diff) .^ 2) .^ 0.5;
    
end