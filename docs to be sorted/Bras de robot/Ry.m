% Matrice rotation selon l'axe Y

function T=Ry(alpha)
T=[cos(alpha) 0 sin(alpha) ;
    0 1 0 ;
    -sin(alpha) 0 cos(alpha)];
end 
