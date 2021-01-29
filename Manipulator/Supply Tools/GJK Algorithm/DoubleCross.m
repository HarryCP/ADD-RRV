function Vector= DoubleCross(VectorA, VectorB, VectorC)
% cross(cross(A, B), C) = B( C'*A ) - A*( C'*B )
    Vector =  VectorB*( VectorC'*VectorA ) - VectorA*( VectorC'*VectorB );

end