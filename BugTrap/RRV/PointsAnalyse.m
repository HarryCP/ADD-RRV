%����PCA���ϰ����еĵ�ķ��������������ײ�ĵ�����ٷ���
function [ Points_free_in_ellipsoid, Judge_node_from_in_ellipsoid] = PointsAnalyse(Coeff, Latent, Mu, Points_free, Num_points_free, Dim, Node_from, P_belif)
    % CoeffΪЭ�������������������������������任����
    % �任��ĵ㣺Score = (X-Mu) * Coeff.
    % LatentΪ������������Ӧ������ֵ���Ӵ�С����
    % MuΪX��ԭ���ݣ����еľ�ֵ
    
    % Points_free_in_ellipsoid ��ʾ����ײ����������ڵ�����ײ�ĵ�
    % Judge_node_from_in_ellipsoid �ж�Node_from�Ƿ�����ײ�����������
    Points_free_in_ellipsoid = [];

    Chisquare_value = chi2inv(P_belif, Dim);        	%��ʾ����ֵ, P_belif��ʾ�������䷶Χ
    Length_axis = Chisquare_value * Latent;             %��ʾ����������᳤��ƽ����
    
    for kk = 1:Num_points_free
        Point_temp = ((Points_free(:, kk)' - Mu) * Coeff)';
        %�Ƚ���任���±�ϵ��
        
        Judge = IsInEllipsoid(Point_temp, Length_axis);
        %�жϵ��Ƿ���������
        
        if Judge == 1
            Points_free_in_ellipsoid(:, end+1) = Points_free(:, kk);
        end
        
    end
    
    Point_temp = ((Node_from' - Mu) * Coeff)';
    Judge_node_from_in_ellipsoid = IsInEllipsoid(Point_temp, Length_axis);
    %�ж�Node_from�Ƿ���������
    
end
    