clc
clear all
close all

%% Valeurs numériques
oa = 0.5;
ab = 1;
bc = 0.3;
oc = 1.2;

% angle de la barre fixe
alphadegree= 75; % [degree] 
alpha=deg2rad(alphadegree);

% valeurs initiales des angles
bethadegree= 300; % [degree] 
betha=deg2rad(bethadegree);

gammadegree= 250; % [degree] 
gamma=deg2rad(gammadegree);

deltadegree= 150; % [degree] 
delta=deg2rad(deltadegree);



%% Boucle vectorielle fermée
% pour la résoudre, il faut itérer plusieurs fois jusqu'à ce que la
% fonction de contrainte Phi soit proche de zéro (epsilon = 0.1)
F_vec = [];
n = 1;
epsilon = 0.1;
for i=1:100
OA = [oa*cos(alpha) oa*sin(alpha)]';
AB = [ab*cos(betha) ab*sin(betha)]';
BC = [bc*cos(gamma) bc*sin(gamma)]';
OC = [oc*cos(delta) oc*sin(delta)]';

% Fonction de contrainte Phi
F=OA+AB+BC-OC;
F_vec = [F_vec, F];
F1=F(1);
F2=F(2);
if abs(F1) < epsilon &&  abs(F2) < epsilon
    disp('DONE')
    break;
end
%% paramètres de contrôle, vecteur des inconnues
% on pilote la direction avec l'angle betha

% le vecteur des inconnues regroupe les deux autres angles 
q=[gamma delta];


%% Calcul du jacobien
dF1dq1 = -bc*sin(gamma);
dF1dq2 = -oc*sin(delta);
dF2dq1 = bc*cos(gamma);
dF2dq2 = oc*cos(delta);

J=[dF1dq1 dF1dq2; dF2dq1 dF2dq2];

%% Méthode de Newton-Raphson

deltaq=-J^(-1)*F;
q2=deltaq+q';

gamma=q2(1);
gammadegree=rad2deg(gamma);
if abs(gammadegree)>360
    gammadegree=mod(gammadegree,360);
    gamma=deg2rad(gammadegree);
end

delta=q2(2);
deltadegree=rad2deg(delta);
if abs(deltadegree)>360
    deltadegree=mod(deltadegree,360);
    delta=deg2rad(deltadegree);
end

end 

F_vec_1 = F_vec(1,1:end);
F_vec_2 = F_vec(2,1:end);
x = linspace(1,length(F_vec));

plot(x,F_vec_1,x,F_vec_2)