%����������ĵ�������з��࣬��Ϊ������ײ�ĺ�����ײ��
function [Points_obstacle, Points_free] = ClassfyPoints(Points_set, Robot_size, Boundary, Obstacle)
    [Dim, Num] = size(Points_set);
    %��ȡ�㼯��ά�Ⱥ�����
    
    Points_obstacle = [];
    Points_free = [];
    
    Flag = zeros(1, Num);
    % ����������ǰ���Ƿ�����ײ
    % 0 ��ʾδ��ײ
    % 1 ��ʾ��ײ
    
    for kk = 1:Num
        Judge = IsCollision(Points_set(:, kk), Robot_size, Boundary, Obstacle);
        %��ÿ���������ײ���
        
        if Judge ~= 0 
            Flag(kk) = 1;
        end
        %�Ե���з���
        
    end
       
    Points_obstacle = Points_set(:, Flag == 1);   
    Points_free = Points_set(:, Flag == 0);
    
end

