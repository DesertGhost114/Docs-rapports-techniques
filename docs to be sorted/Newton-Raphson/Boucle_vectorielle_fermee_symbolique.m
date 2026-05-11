clc
clear all
close all

syms alpha betha gamma delta oa ab bc co
%% Valeurs numériques

oa_num = 0.5;
ab_num = 1;
bc_num = 0.3;
co_num = 1.2;

% angle de la barre fixe
alphadegree= 75; % [degree] 
alpha_num=deg2rad(alphadegree);

% valeurs initiales des angles
bethadegree= 300; % [degree] 
betha_num=deg2rad(bethadegree);

gammadegree= 250; % [degree] 
gamma_num=deg2rad(gammadegree);

deltadegree= 150; % [degree] 
delta_num=deg2rad(deltadegree);



%% Boucle vectorielle fermée
% on établit les formules sysmboliques

OA = [oa*cos(alpha) oa*sin(alpha)]';
AB = [ab*cos(betha) ab*sin(betha)]';
BC = [bc*cos(gamma) bc*sin(gamma)]';
CO = [co*cos(delta) co*sin(delta)]';

% Fonction de contrainte Phi : on fait un parcours qui commence au point O,
% passe par tous les points et revient à ce point de départ.
Phi=OA+AB+BC+CO;
% logiquement, la fonction Phi doit être nulle. 
 
Phi1=Phi(1);
Phi2=Phi(2);

%% paramètres de contrôle, vecteur des inconnues
% on pilote la direction avec l'angle betha

% le vecteur des inconnues regroupe les deux autres angles 
q=[gamma delta];


%% Calcul du jacobien
% dPhi1dq1 = -bc*sin(gamma);
dPhi1dq1 = diff(Phi1,gamma);
% dPhi1dq2 = -co*sin(delta);
dPhi1dq2 = diff(Phi1,delta);
% dPhi2dq1 = bc*cos(gamma);
dPhi2dq1 = diff(Phi2,gamma);
% dPhi2dq2 = co*cos(delta);
dPhi2dq2 = diff(Phi2,delta);

J=[dPhi1dq1 dPhi1dq2; dPhi2dq1 dPhi2dq2];

%% Méthode de Newton-Raphson

deltaq=-J^(-1)*Phi;
% nouvelle valeur des angles gamma et delta
q2=deltaq+q';

% évaluer avec les valeurs numériques
oa = oa_num;
ab = ab_num;
bc = bc_num;
co = co_num;
% valeurs de départ des angles
alpha = alpha_num;
betha = betha_num;
gamma = gamma_num;
delta = delta_num;

Phi_num = eval(Phi);
% La fonction de contrainte Phi n'est pas nulle car les angles ne sont pas
% corrects et sont juste une approximation.
% La solution est obtenue quand Phi est nulle. Pour cela, il faut itérer 
% plusieurs fois avec la méthode de Newton-Raphson. 
n = 10;
for i=1:n
    % évaluer avec les valeurs numériques
    oa = oa_num;
    ab = ab_num;
    bc = bc_num;
    co = co_num;
    % valeurs de départ des angles
    alpha = alpha_num;
    betha = betha_num;
    gamma = gamma_num;
    delta = delta_num;
    
    q2_num = eval(q2);
    Phi_num = eval(Phi);

    
    
    gamma_num=q2_num(1);
    delta_num=q2_num(2);
    
    % les angles doivent être positifs, dans tout les cas
    if gamma_num <0
        gamma_num = 360+gamma_num;
    end
    if delta_num <0
        delta_num = 360+delta_num;
    end
    
    % gamma doit être entre pi et 3pi/2°
%     if (gamma>3*pi/2) || (gamma < pi) 
%         gamma=mod(gammadegree,360);
%         gamma_num=deg2rad(gammadegree);
%     end
    gammadegree=rad2deg(gamma_num);

    % delta doit être entre pi/2 et pi
%     if deltadegree>360
%         deltadegree=mod(deltadegree,360);
%         delta_num=deg2rad(deltadegree);
%     end
    deltadegree=rad2deg(delta);


end 
