%��ʾ�������ϵ��,��X����ת
%��ת�Ƕ�Ϊtheta,��λ��rad
function Rx = RotX(theta)
    Cx =[ 1           0            0;
          0  cos(theta)  -sin(theta);
          0  sin(theta)   cos(theta) ];
      
    O = zeros(3, 1);
    
    Rx = [ Cx, O;
           O', 1];
       
       
end

