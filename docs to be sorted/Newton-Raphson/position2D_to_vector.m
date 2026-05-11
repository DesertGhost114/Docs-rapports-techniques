function vector=position2D_to_vector(a,b,N) % angle in rad

slope = (b(2)-a(2))/(b(1)-a(1));

step = (b(1)-a(1))/(N-1);

vector(1,1) = a(1);
vector(1,2) = a(2);

for i=2:N
    vector(i,1) = vector(i-1,1)+ step;
    vector(i,2) = vector(i-1,2)+ step * slope;
end

end
