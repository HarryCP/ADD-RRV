%表示齐次坐标系下,绕X轴旋转
%旋转角度为theta,单位是rad
function Rx = RotX(theta)
    Cx =[ 1           0            0;
          0  cos(theta)  -sin(theta);
          0  sin(theta)   cos(theta) ];
      
    O = zeros(3, 1);
    
    Rx = [ Cx, O;
           O', 1];
       
       
end

