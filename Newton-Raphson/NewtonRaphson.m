clc
clear all
close all
%% charger le schéma

imshow('schema.png')
%% Valeurs numériques
% la=0.5; 
l_a=5; % [m]
l_ab=6;% [m]
l_bc=5; % [m]
l_c=10;% [m]
% alpha_degree= [30,40,50,60]'; % [degree] 
alpha_degree = [40,50,60,70];
alpha=deg2rad(alpha_degree);

beta_degree=zeros(1,length(alpha)); % [degree]
gamma_degree=zeros(1,length(alpha)); % [degree]
for i=1:length(alpha)
    beta_degree(i)=170;
    gamma_degree(i)=150;
end
beta=deg2rad(beta_degree);
gamma=deg2rad(gamma_degree);
%% Boucle vectorielle fermée
% for j=1:size(alpha)
% for i=1:50
% % OA+AB+BC+CO=0  
% O = [0 0]';
% OA=[l_a*cos(alpha(j)) l_a*sin(alpha(j))]';
% AB=[l_ab*cos(beta(j)) l_ab*sin(beta(j))]';
% BC=[-l_bc*cos(gamma(j)) l_bc*sin(gamma(j))]';
% CO=[-l_c 0]';
% % Fonction de contrainte
% F=OA+AB+BC+CO;
% F1=F(1);
% F2=F(2);
% %% paramètres de contrôle, vecteur des inconnues
% % on pilote alpha
% % le vecteur des inconnues est 
% q=[beta(j) gamma(j)]'; % q=[q_2 q_3]
% %% Calcul du jacobien
% dF1dq2=-l_ab*sin(beta(j));
% dF1dq3=-l_bc*sin(gamma(j));
% dF2dq2=l_ab*cos(beta(j));
% dF2dq3=l_bc*cos(gamma(j));
% 
% J=[dF1dq2 dF1dq3; dF2dq2 dF2dq3];
% 
% %% Méthode de Newton-Raphson
% 
% deltaq=-J^(-1)*F;
% q=deltaq+q;
% 
% beta(j)=q(1);
% gamma(j)=q(2);
% 
% end
% end
for j=1:length(alpha)
    
for i=1:20
% OA+AB+BC+CO=0  
O = [0 0]';
OA(1,j)=l_a*cos(alpha(j));
OA(2,j)=l_a*sin(alpha(j));
AB(1,j)=l_ab*cos(beta(j));
AB(2,j)=l_ab*sin(beta(j));
BC(1,j)=-l_bc*cos(gamma(j));
BC(2,j)=l_bc*sin(gamma(j));
CO(1,j)=-l_c;
CO(2,j)= 0;
% Fonction de contrainte
F=OA(:,j)+AB(:,j)+BC(:,j)+CO(:,j);
F1(j)=F(1);
F2(j)=F(2);
%% paramètres de contrôle, vecteur des inconnues
% on pilote alpha
q1 = alpha;
% le vecteur des inconnues est 
q=[beta(j) gamma(j)]'; % q=[q_2 q_3]
%% Calcul du jacobien
dF1dq2=-l_ab*sin(beta(j));
dF1dq3=l_bc*sin(gamma(j));
dF2dq2=l_ab*cos(beta(j));
dF2dq3=- l_bc*cos(gamma(j));

J=[dF1dq2 dF1dq3; dF2dq2 dF2dq3];

%% Méthode de Newton-Raphson

deltaq=-J^(-1)*F;
q=deltaq+q;

beta(j)=q(1);
gamma(j)=q(2);

end
end

% compute the position again
OB = OA + AB;
OC = OA + AB + BC;
OA_vec=[];
AB_vec=[];
BC_vec=[];
CO_vec=[];
for i=1:size(OA,2)
a = position2D_to_vector(O,OA(:,i),10);
OA_vec = [OA_vec, a];
a = position2D_to_vector(OA(:,i),OA(:,i)+AB(:,i),10);
AB_vec = [AB_vec, a];
a = position2D_to_vector(OA(:,i)+AB(:,i),OA(:,i)+AB(:,i)+BC(:,i),10);
BC_vec = [BC_vec, a];
a = position2D_to_vector(OA(:,i)+AB(:,i)+BC(:,i),O,10);
CO_vec = [CO_vec, a];
end 
figure
subplot(3,1,1)
plot(OA_vec(:,1),OA_vec(:,2),AB_vec(:,1),AB_vec(:,2),BC_vec(:,1),BC_vec(:,2),CO_vec(:,1),CO_vec(:,2))
title('\alpha = 30°')
legend('OA','AB','BC','CO','Location','bestoutside')
subplot(3,1,2)
plot(OA_vec(:,3),OA_vec(:,4),AB_vec(:,3),AB_vec(:,4),BC_vec(:,3),BC_vec(:,4),CO_vec(:,3),CO_vec(:,4))
title('\alpha = 40°')
legend('OA','AB','BC','CO','Location','bestoutside')
subplot(3,1,3)
plot(OA_vec(:,5),OA_vec(:,6),AB_vec(:,5),AB_vec(:,6),BC_vec(:,5),BC_vec(:,6),CO_vec(:,5),CO_vec(:,6))
title('\alpha = 50°')
legend('OA','AB','BC','CO','Location','bestoutside')

%% 
% les angles sont définis entre 0 et 2pi, il faut donc changer les angles
% pour qu'ils soient compris dans cet intervalle. il faut d'abord savoir
% dans quels cadrans se trouve chaque angle, pour donner la valeur exacte.
% pour controler ceci, la BCF doit être recalculée. 

% on utilise alors la fonction
% de division pour respecter cet intervalle.
% beta_rad = zeros(4,1);
% gamma_rad = zeros(4,1);
for j=1:length(beta)
if abs(beta(j)) > 2*pi
        beta_rad(j)=mod(beta(j),2*pi);
end
if beta_rad(j) < 0
   beta_rad(j) = 2*pi - beta_rad(j);
end
end

for j=1:length(gamma)
if abs(gamma(j)) > 2*pi
    gamma_rad(j)=mod(gamma(j),2*pi);
end
if gamma_rad(j) < 0
	gamma_rad(j) = 2*pi - gamma_rad(j);
end
end
beta_degree=rad2deg(beta_rad);
gamma_degree=rad2deg(gamma_rad);

%% Plot du quadrilatère

figure
plot(alpha_degree,beta_degree,'-x',alpha_degree,gamma_degree,'-o')
xlabel('\alpha [°]')
ylabel('Angle [°]')
legend('\beta [°]','\gamma [°]')
