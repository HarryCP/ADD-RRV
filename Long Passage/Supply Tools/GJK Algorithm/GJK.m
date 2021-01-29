function Judge = GJK(ShapeA, FlagA, ShapeB, FlagB)
%�Ը���������͹��ShapeA��ShapeB������ײ���
%Ŀǰ��������2ά�����
%��Falg=0����ʾ��״ΪԲ��Shape��һ�б�ʾԲ�����꣬�ڶ��б�ʾ�뾶������һ��Ԫ�ض��ǰ뾶��ֵ��
%��Flag=1����ʾ��״Ϊ͹����Σ�Shapeÿһ�б�ʾһ����������
	
Judge = 0;
    
    [m1, n1] = size(ShapeA);
    [m2, n2] = size(ShapeB);
    % m��ʾ��״��ά�ȣ�n��ʾ����Σ������壩�Ķ����� 
    
    if m1 ~= m2
        disp('ERROR: Wrong Dimension In GJK !!!');
        return;
    end
    %��֤������ײ��������ͼ��ά����ͬ
    
    Direction = rand(m1, 1);
    %Ѱ��һ���������
    Simplix = repmat(Support(ShapeA, FlagA, ShapeB, FlagB, Direction), 1, m1+1); 
    %�õ��������еĵ�һ����
    Direction = -Direction;
    %��������������Ϊ�˵õ���һ����
    k = 1;
    %��ʾ���ĵ������
    
    while true
        k = k + 1;
        Simplix(:, 1) = Support(ShapeA, FlagA, ShapeB, FlagB, Direction);
        %�򵥴��������һ���µ�
        if Simplix(:,1)'*Direction < 0
            Judge = 0;
            return;
            %�������ӵĵ��Vector������û�д���ԭ�㣬
            %��ô�����ܷ�����ײ
            %(��Ϊ����ӵĵ���Minkowski Difference�ı߽���)
        else
            [Judge , Simplix, Direction] = IsContainOrigin(Simplix, Direction, k);
            %�жϵ������Ƿ����ԭ��
            %�������ΰ���ԭ�㣬������ײ
            %���򣬻�����ȷ���Ƿ�����ײ��
           	%Ѱ����ԭ������������ߣ�������������ָ��ԭ��ķ�����������Ϊ��������
            if Judge == 1
                return;
                %�������ΰ���ԭ�㣬������ײ
            end
           
        end
         
    end
   
end
