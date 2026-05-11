function My=Roty(angle) % angle in rad

My=[cos(angle),0,-sin(angle);
    0,1,0;
    sin(angle),0,cos(angle)];
end
