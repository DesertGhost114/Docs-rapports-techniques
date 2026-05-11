clc
clear all
close all

syms w l d betha deltai deltao phi2

%% Valeurs numériques
% r=0.5; % r=w/l
% w=2.4; % [m]
% l=w/r;% [m]
% d=0.4; % [m]
% bethadegree= 6; % [degree] 10 14 18 22
% betha=deg2rad(bethadegree);
% % angle de braquage initial
% deltaidegree=0;
% deltai=deg2rad(deltaidegree);
% % angle de la barre BC initial
% phi2=0;

%% Condition d'Ackermann : garantir le non-glissement des roues
% cot(deltao)-cot(deltai)=w/l

% deltao=acot(w/l* cot(deltai));

%% Boucle vectorielle fermée

% AB+BC+CD+DA à mettre dans le référentiel du buggy. Ainsi les réponses
% seront données dans ce repère. 

% Sans braquer les roues, la BVF permet de trouver s, la longueur de la barre BC :
s=w-2*sin(betha);

% on définit les angles
phi1=3*pi/4+betha+deltai;
phi3=3*pi/4-betha+deltao;

% On braque la roue intérieure d'un angle deltai :

AB=[d*sin(betha) -d*cos(betha)]';
BC=[s*cos(phi2) -s*sin(phi2)]';
CD=[-d*sin(phi3) d*cos(phi3)]';
DA=[-w 0]';

% Fonction de contrainte

F=AB+BC+CD+DA;
 
F1=F(1);
F2=F(2);


%% paramètres de contrôle, vecteur des inconnues
% on pilote la direction avec l'angle delati

% le vecteur des inconnues est 
q=[deltao phi2];


%% Calcul du jacobien

dF1dq1=diff(F1,q(1));
dF1dq2=diff(F1,q(2));
dF2dq1=diff(F2,q(1));
dF2dq2=diff(F2,q(2));

J=[dF1dq1 dF1dq2; dF2dq1 dF2dq2];

%% Méthode de Newton-Raphson

deltaq = -J^(-1)*F;

q2=q'+deltaq