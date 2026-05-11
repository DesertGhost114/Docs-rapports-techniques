% Matrice rotation selon l'axe X

function T=Rx(alpha)
T=[1 0 0 ;
    0 cos(alpha) -sin(alpha);
    0 sin(alpha) cos(alpha)];
end 
