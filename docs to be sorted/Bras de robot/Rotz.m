function Mz=Rotz(angle) % angle in rad

Mz=[cos(angle),-sin(angle),0;
    sin(angle),cos(angle),0;
    0,0,1];
end
