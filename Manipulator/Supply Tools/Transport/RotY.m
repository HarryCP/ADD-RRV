%��ʾ�������ϵ��,��Y����ת
%��ת�Ƕ�Ϊtheta,��λ��rad

function Ry = RotY(theta)
    Cy = [ cos(theta)  0   sin(theta);
                    0  1            0;
          -sin(theta)  0   cos(theta) ];

    O = zeros(3, 1);
    
    Ry = [ Cy, O;
           O', 1];
       
end