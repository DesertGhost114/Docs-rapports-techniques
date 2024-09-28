clc
clear all
close all
%% Modélisation d'un bras de robot
% L'architecture est faite en série, ce qui veut dire que le mouvement
% d'une partie du robot est directement influencée par le mouvement de
% l'élément précédent. Par exemple, le mouvement du bras est lié au
% mouvement de l'avant-bras. 
syms l1 l2 l3 alpha1 alpha2 alpha3 betha1 betha2 betha3 gamma1 gamma2 gamma3
%% Schéma du bras
% imshow('Bras_robot.png')

% OA : avant-bras
% AB : bras
% BC : main

%% Mouvement de l'avant-bras : l'épaule possède trois degrés de libertés

lab=transpose([0 0 -l1]);
OA = Rx(alpha1)*Ry(alpha2)*Rz(alpha3)*lab;

%% Mouvement du bras : le coude a un degré de libertés. 

lb=transpose([0 0 -l2]);

% Mouvement du bras dans le repère local
AB1 = Rx(betha1)*lb;
% Il faut prendre en compte le mouvement réalisé par l'élément précédent :
% Mouvement du bras dans le repère global
AB = Rx(alpha1)*Ry(alpha2)*Rz(alpha3)*AB1;

% Position du poignet par rapport au repère global
OB=OA+AB;

%% Mouvement de la main : 2 DDL + rotation bras
lm=transpose([0 0 -l3]);
% Mouvement du bras dans le repère local
BC1=Rx(gamma1)*Ry(gamma2)*Rz(gamma3)*lm;
% Mouvement du bras dans le repère global
BC=Rx(alpha1)*Ry(alpha2)*Rz(alpha3)*Rx(betha1)*BC1;

% Position bout de la main
OC = OA+AB+BC;

%% résolution
% on part d'une position de départ, avec des valeurs connues pour les
% angles et la longueur des bras. On veut atteindre une certainte position
% avec le bout du bras du robot, c'est-à-dire qu'on impose OC.il y a 
% plusieurs manières de résoudre :
% - Boucle vectorielle fermée et Newton-Raphson
% - Nessus
% On peut améliorer la résolution en l'optimisant :
% - arriver le plus rapidement possible à la position souhaité
% - minimiser la variation d'angle pour chaque articulation <=> minimum de
% mouvements


%% Valeur numérique

l1=1;
l2=0.8;
l3=0.4;
alpha1=0/360*2*pi;
alpha2=0/360*2*pi;
alpha3=0/360*2*pi;
betha1=45/360*2*pi;
gamma1=0/360*2*pi;
gamma2=0/360*2*pi;
gamma3=0/360*2*pi;
OAn=eval(OA);
ABn=eval(AB);
BCn=eval(BC);
OCn=eval(OC);
O=transpose([0 0 0]);
P = [O,OAn, ABn, BCn];
x = P(1,:);
y = P(2,:);
z = P(3,:);
plot3(x,y,z,'g')

% plot plan xz
plot(x,z)