%表示齐次坐标系下,绕Y轴旋转
%旋转角度为theta,单位是rad

function Ry = RotY(theta)
    Cy = [ cos(theta)  0   sin(theta);
                    0  1            0;
          -sin(theta)  0   cos(theta) ];

    O = zeros(3, 1);
    
    Ry = [ Cy, O;
           O', 1];
       
end