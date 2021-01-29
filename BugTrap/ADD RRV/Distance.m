function Dist = Distance(From, To)
    
    Dist =  sum((From - To) .^ 2) .^ 0.5;
    
end