%�������ε�����λ�á���������������������
function Rec_pos = ShapeToPoint(Center, Length, Width)
    Half_length = Length/2;
    Half_width = Width/2;
    
    Rec_pos = [ Center(1)+Half_length, Center(1)+Half_length, Center(1)-Half_length, Center(1)-Half_length, Center(1)+Half_length;
                Center(2)+Half_width,  Center(2)-Half_width,  Center(2)-Half_width,  Center(2)+Half_width,  Center(2)+Half_width;
                0,                     0,                     0,                     0,                     0;
                1                      1,                     1,                     1,                     1];
       
end