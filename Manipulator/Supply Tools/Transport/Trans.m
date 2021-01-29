%表示齐次坐标下平移变换
%p表示平移列向量

function Tra = Trans(p)
    E = eye(3);
    O = zeros(1, 3);
    Tra = [ E, p;
            O, 1 ];
end