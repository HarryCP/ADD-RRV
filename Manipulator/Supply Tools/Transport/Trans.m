%��ʾ���������ƽ�Ʊ任
%p��ʾƽ��������

function Tra = Trans(p)
    E = eye(3);
    O = zeros(1, 3);
    Tra = [ E, p;
            O, 1 ];
end