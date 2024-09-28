clc
clear all
close all

syms betha alpha theta gamma w l d 


%% Numerical values

betha_degree = [6 10 14 22 19]';
% betha_num = deg2rad(betha_degree_num); % 10 14 18 22°
betha_num = deg2rad(betha_degree);

w_num = 2.4;
l_num = 4.8;
d_num = 0.4;
% theta is the controlled variable
% the theta angle is linked with betha and delta_i
% the alpha angle is linked with betha and delta_o
% minimal value of theta, for different values of betha
theta_o_degree = 270 + betha_degree;

theta_o_degree_max = theta_o_degree+60;
% 
theta_degree_vec = zeros(length(betha_degree),12);
theta_degree_vec(:,1)=theta_o_degree;
for i=1:length(betha_degree)
    theta_degree_vec(i,1)=theta_o_degree(i);
for j=2:length(theta_degree_vec)
    theta_degree_vec(i,j)=theta_degree_vec(i,j-1)+5;
end
end

theta_num = deg2rad(theta_degree_vec);
% initial values for the unknown variables
alpha_degree_ini = 300;
gamma_degree_ini = 175;
% after the iteration, we will obtain the correct ones.

% delta_i is the steering angle of the interior wheel. 
% we will compute its value after getting the values of theta
% delta_i = theta - betha - 3*pi/2;
% we will solve the problem with theta, then compute delta_i and delta_o

% for each value of theta, we will have a specified value for alpha and
% gamma. they have the same vector size. 
% because we have different values for gamma and betha, we create arrays 
% for the unkown variables. after the computations, the solutions will be
% saved in these arrays : each column is a different value for delta_i, and
% each line is a different value for betha. 

alpha_num = zeros(size(theta_num,1),size(theta_num,2));
gamma_num = zeros(size(theta_num,1),size(theta_num,2));

for i=1:size(alpha_num,1)
    for j=1:size(alpha_num,2)
        alpha_num(i,j) = deg2rad(alpha_degree_ini);
        gamma_num(i,j) = deg2rad(gamma_degree_ini);
    end
end


%% Other variables
% Condition of Ackermann
% delta_o = acot(w/l + cot(delta_i));

% delta_i = theta - betha - deg2rad(270);
% delta_o = alpha + betha - deg2rad(270);
m = w - 2 * d*sin(betha);

%% Boucle vectorielle fermée
% on établit les formules sysmboliques

OA = [w 0]';
AB = [d*cos(alpha) d*sin(alpha)]';
BC = [m*cos(gamma) m*sin(gamma)]';
CO = [-d*cos(theta) -d*sin(theta)]';

% Fonction de contrainte Phi : on fait un parcours qui commence au point O,
% passe par tous les points et revient à ce point de départ.
Phi=OA+AB+BC+CO;
% logiquement, la fonction Phi doit être nulle. 
 
Phi1=Phi(1);
Phi2=Phi(2);

%% paramètres de contrôle, vecteur des inconnues
% on pilote la direction avec l'angle betha

% le vecteur des inconnues regroupe les deux autres angles 
q=[alpha gamma];
q1 = q(1);
q2 = q(2);

%% Calcul du jacobien
dPhi1dq1 = diff(Phi1,q1);
% dPhi1dq2 = -co*sin(delta);
dPhi1dq2 = diff(Phi1,q2);
% dPhi2dq1 = bc*cos(gamma);
dPhi2dq1 = diff(Phi2,q1);
% dPhi2dq2 = co*cos(delta);
dPhi2dq2 = diff(Phi2,q2);

J=[dPhi1dq1 dPhi1dq2; dPhi2dq1 dPhi2dq2];

%% Méthode de Newton-Raphson

deltaq=-J^(-1)*Phi;
% new values for the variables
q2=deltaq+q';

%% evaluate with numerical values
% we will solve the problem taking into account different values of angles
% for betha and delta_i

% for different values of betha
for j=1:size(theta_num,1)

% for different values of theta
for k=1:size(theta_num,2)
    
betha = betha_num(j);
w = w_num;
l=l_num;
d = d_num;
theta = theta_num(j,k);
alpha = alpha_num(j,k);
gamma = gamma_num(j,k);
q_num = eval(q);

% Phi_num = eval(Phi);
% deltaq_num = eval(deltaq);
% q2_num = eval(q2);
% La fonction de contrainte Phi n'est pas nulle car les angles ne sont pas
% corrects et sont juste une approximation.
% La solution est obtenue quand Phi est nulle. Pour cela, il faut itérer 
% plusieurs fois avec la méthode de Newton-Raphson. 
n = 5;
for i=1:n
    
    Phi_num = eval(Phi);
    deltaq_num = eval(deltaq);
    q2_num = eval(q2);
    % alpha must be between 
    alpha = q2_num(1);
    % gamma must be between pi/2 and pi
    gamma = q2_num(2);

    % convert angles from grad to degree
%     delta_i_degree=rad2deg(delta_i);
%     gamma_degree=rad2deg(gamma);
    
%     
%     gamma_num=q2_num(1);
%     delta_num=q2_num(2);
%     
%     % les angles doivent être positifs, dans tout les cas
%     if gamma_num <0
%         gamma_num = 360+gamma_num;
%     end
%     if delta_num <0
%         delta_num = 360+delta_num;
%     end
%     
%     % gamma doit être entre pi et 3pi/2°
% %     if (gamma>3*pi/2) || (gamma < pi) 
% %         gamma=mod(gammadegree,360);
% %         gamma_num=deg2rad(gammadegree);
% %     end
%     gammadegree=rad2deg(gamma_num);
% 
%     % delta doit être entre pi/2 et pi
% %     if deltadegree>360
% %         deltadegree=mod(deltadegree,360);
% %         delta_num=deg2rad(deltadegree);
% %     end
%     deltadegree=rad2deg(delta);


end 
    % we save the solutions into the arrays
    alpha_num(j,k) = q2_num(1);
    gamma_num(j,k) = q2_num(2);

end

end


alpha_degree_vec = rad2deg(alpha_num);

gamma_degree_vec = rad2deg(gamma_num);

%% Angles of steering for the interior and the exterior wheels
% Condition of Ackermann
delta_i_Ackermann_degree = linspace(0,60,9);
delta_i_Ackermann = deg2rad(delta_i_Ackermann_degree);
delta_o_Ackermann = acot(w/l + cot(delta_i_Ackermann));
for i=1:length(delta_o_Ackermann)
    if delta_o_Ackermann(i)<0
        delta_o_Ackermann(i)=delta_o_Ackermann(i)+pi/2;
    end
end

delta_o_Ackermann_degree = rad2deg(delta_o_Ackermann);

figure
plot(delta_i_Ackermann_degree,delta_o_Ackermann_degree)
xlabel('\delta_i [°]')
ylabel('\delta_o [°]')
legend('Condition of Ackermann')

% We will compare these values to the ones obtained with the solutions
% delta_i = theta - betha - 3*pi/2;
% delta_o = alpha + betha - 3*pi/2;
delta_i = zeros(size(theta_num,1),size(theta_num,2));
delta_o = zeros(size(theta_num,1),size(theta_num,2));
for i=1:size(theta_num,1)
    for j=1:size(theta_num,2)
        delta_i(i,j)=theta_num(i,j)-betha_num(i)-3*pi/2;
        delta_o(i,j)=alpha_num(i,j)+betha_num(i)-3*pi/2;
    end
end
delta_i_degree_vec = rad2deg(delta_i);

delta_o_degree_vec = rad2deg(delta_o);

figure
plot(delta_i_Ackermann_degree,delta_o_Ackermann_degree,'-0-x',delta_i_degree_vec(1,:),delta_o_degree_vec(1,:),delta_i_degree_vec(2,:),delta_o_degree_vec(2,:),delta_i_degree_vec(3,:),delta_o_degree_vec(3,:),delta_i_degree_vec(4,:),delta_o_degree_vec(4,:),delta_i_degree_vec(5,:),delta_o_degree_vec(5,:))
xlabel('\delta_i [°]')
ylabel('\delta_o [°]')
legend('Condition of Ackermann',['\beta =', num2str(betha_degree(1)),'°'],['\beta =', num2str(betha_degree(2)),'°'],['\beta =', num2str(betha_degree(3)),'°'],['\beta =', num2str(betha_degree(4)),'°'],['\beta =', num2str(betha_degree(5)),'°'])

% Analysis, the best betha value is 22° until delta_i = 20. 
% then it convergs to betha 14°.
% to find the best angle for betha, we do several iterations with, for
% example, betha values of 16,18 and 20. until we find the most suited one.

%% Instantneous rotation center
% the origin is the center of the rear axle. 
% D1 
% x = -w/2-m*sin(pi/2-delta_i)
% y = l-m*cos(pi/2-delta_i)
% y = 0 -> m = l/cos(pi/2-delta_i)
m1 = l/cos(pi/2-delta_i(1,2));
% position of the IRC
X_IRC = -w/2 - m1 + sin(pi/2-delta_i(1,2));
% verification with D2 
% x = w/2-m*sin(pi/2-delta_o)
% y = l-m*cos(pi/2-delta_o)
s1 = l/cos(pi/2-delta_o(1,2));
% position of the IRC
X_IRC_2 = w/2 - s1 + sin(pi/2-delta_o(1,2));
%% Draw the quadrilateral of Jeantaud
% origin
O=[0 0];
% OA 
OA_num = eval(OA);
% OA_vec = [[0 0]' OA_num];
x = linspace(O(1),OA_num(1),5);
y = linspace(O(2),OA_num(2),5);
OA_vec=[x;y];

betha = betha_num(1);
alpha = alpha_num(1,1);
gamma = gamma_num(1,1);
theta = theta_num(1,1);

AB_vec_num = zeros(length(betha_num),length(delta_i_degree),2);
BC_vec_num = zeros(length(betha_num),length(delta_i_degree),2);
CO_vec_num = zeros(length(betha_num),length(delta_i_degree),2);

for i=1:length(betha_num)
for j=1:length(delta_i_num)
    
    betha = betha_num(i);
    delta_i = delta_i_num(j);
    alpha = alpha_num(i,j);
    gamma = gamma_num(i,j);
    theta = theta_num(i,j);

    AB_num = eval(AB);
    OB_num = OA_num + AB_num;
    AB_vec = [OA_num OB_num];
    AB_vec_num(i,j,1) = AB_vec(1);
    AB_vec_num(i,j,2) = AB_vec(2);

    BC_num = eval(BC);
    OC_num = OB_num + BC_num;
    BC_vec = [OB_num OC_num];
    BC_vec_num(i,j,1) = BC_vec(1);
    BC_vec_num(i,j,2) = BC_vec(2);

    CO_num = eval(CO);
    CO_vec = [[0 0]' -CO_num];
    CO_vec_num(i,j,1) = CO_vec(1);
    CO_vec_num(i,j,2) = CO_vec(2);

end
end



% x : (1,1:end,1)
% y : (1,1:end,2)
figure 
plot(OA_vec(1:end,1),OA_vec(1:end,1),AB_vec_num(1,1:end,1),AB_vec_num(1,1:end,2),BC_vec_num(1,1:end,1),BC_vec_num(1,1:end,2),OC_vec_num(1,1:end,1),OC_vec_num(1,1:end,2))
title(['Quadrilater of Jeantaud with \beta =', num2str(betha_degree(1)),'°'])
legend('OA','AB','BC','OC','Location','best')

% left wheel
l_wheel_x = [-sin(delta_i) 0 sin(delta_i)];
l_wheel_y = [cos(delta_i) 0 -cos(delta_i)];

% right wheel
r_wheel_x = [w-sin(delta_o_num) w w+sin(delta_o_num)];
r_wheel_y = [cos(delta_o_num) 0 -cos(delta_o_num)];


figure 
plot(OA_vec(1,1:end),OA_vec(2,1:end),AB_vec(1,1:end),AB_vec(2,1:end),BC_vec(1,1:end),BC_vec(2,1:end),OC_vec(1,1:end),OC_vec(2,1:end),l_wheel_x,l_wheel_y,r_wheel_x,r_wheel_y)
title(['Quadrilater of Jeantaud with \beta =', num2str(betha_degree(1)),'°'])
legend('OA','AB','BC','OC','Left wheel','Right wheel','Location','best')


% 
% % OA = [w 0]';
% % AB = [d*cos(alpha) d*sin(alpha)]';
% % BC = [m*cos(gamma) m*sin(gamma)]';
% % CO = [d*cos(theta) d*sin(theta)]';
% 
% OA_num = eval(OA);
% OA_vec = [[0 0]' OA_num];
% 
% AB_num = eval(AB);
% AB_vec = [[0 0]' AB_num];
% 
% BC_num = eval(BC);
% BC_vec = [[0 0]' BC_num];
% 
% CO_num = eval(eval(CO));
% CO_vec = [[0 0]' CO_num];
% 
% figure 
% plot(OA_vec(1,1:end),OA_vec(2,1:end),AB_vec(1,1:end),AB_vec(2,1:end),BC_vec(1,1:end),BC_vec(2,1:end),CO_vec(1,1:end),CO_vec(2,1:end))
% legend('OA','AB','BC','CO','Location','best')




% complute delta_i and delta_o
% delta_i = theta - betha - deg2rad(270);
% delta_o = acot(w/l + cot(delta_i));
% % compute delta_o : the steering angle of the exterior wheel

% delta_o_vec=[];
% delta_o_degree_vec=[];
% for i=1:length(delta_i_vec)
%     delta_o_vec(i) = acot(w/l + cot(delta_i_vec(i)));
%     delta_o_degree_vec(i)=rad2deg(delta_o_vec(i));
% end
% 

% 
% figure
% plot(delta_i_degree_vec,delta_o_degree_vec)
% % legend('\delta_i','\delta_o','\gamma','Location','best')
% xlabel('\delta_i')
% ylabel('\delta_o')
% 