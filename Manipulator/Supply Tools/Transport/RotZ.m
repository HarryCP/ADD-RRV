%��ʾ�������ϵ��,��Y����ת
%��ת�Ƕ�Ϊtheta,��λ��rad
function Rz = RotZ(theta)
    Cz = [  cos(theta)  -sin(theta)  0;
       	    sin(theta)   cos(theta)  0;
                     0           0   1 ];

	O = zeros(3, 1);
    
    Rz = [ Cz, O;
           O', 1];
       
end