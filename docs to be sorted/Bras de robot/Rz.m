% Matrice rotation selon l'axe Z

function T=Rz(alpha)
T=[cos(alpha) -sin(alpha) 0;
    sin(alpha) cos(alpha) 0;
    0 0 1];
end 
