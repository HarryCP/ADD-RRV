function Point = FarthestPoint(Shape, Flag, Vector)
	%����״Shape�ڵõ�Vector��������Զ�Ķ���
	%��Falg=0����ʾ��״ΪԲ��Shape��һ�б�ʾԲ�����꣬�ڶ��б�ʾ�뾶������һ��Ԫ�ض��ǰ뾶��ֵ��
	%��Flag=1����ʾ��״Ϊ͹����Σ�Shapeÿһ�б�ʾһ����������
    if Flag == 1
        [~, n] = max(Vector' * Shape);
        Point = Shape(:, n);
    else
        if Flag == 0
            Point = Shape(:, 1) + Shape(1, 2)*Vector/norm(Vector);
        else
           fprint('ERROR: Wrong Shape In FarthestPoint of GJK !!!'); 
        end
    end
        
end
