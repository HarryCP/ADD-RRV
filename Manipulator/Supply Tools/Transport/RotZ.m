%表示齐次坐标系下,绕Y轴旋转
%旋转角度为theta,单位是rad
function Rz = RotZ(theta)
    Cz = [  cos(theta)  -sin(theta)  0;
       	    sin(theta)   cos(theta)  0;
                     0           0   1 ];

	O = zeros(3, 1);
    
    Rz = [ Cz, O;
           O', 1];
       
end