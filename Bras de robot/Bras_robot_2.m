clc
clear all
close all
%% Mod�lisation d'un bras de robot
% L'architecture est faite en s�rie, ce qui veut dire que le mouvement
% d'une partie du robot est directement influenc�e par le mouvement de
% l'�l�ment pr�c�dent. Par exemple, le mouvement du bras est li� au
% mouvement de l'avant-bras. 
syms l1 l2 l3 alpha1 alpha2 alpha3 betha1 betha2 betha3 gamma1 gamma2 gamma3
%% Sch�ma du bras
% imshow('Bras_robot.png')

% OA : avant-bras
% AB : bras
% BC : main

%% Mouvement de l'avant-bras : l'�paule poss�de trois degr�s de libert�s

lab=transpose([0 0 -l1]);
OA = Rx(alpha1)*Ry(alpha2)*Rz(alpha3)*lab;

%% Mouvement du bras : le coude a un degr� de libert�s. 

lb=transpose([0 0 -l2]);

% Mouvement du bras dans le rep�re local
AB1 = Rx(betha1)*lb;
% Il faut prendre en compte le mouvement r�alis� par l'�l�ment pr�c�dent :
% Mouvement du bras dans le rep�re global
AB = Rx(alpha1)*Ry(alpha2)*Rz(alpha3)*AB1;

% Position du poignet par rapport au rep�re global
OB=OA+AB;

%% Mouvement de la main : 2 DDL + rotation bras
lm=transpose([0 0 -l3]);
% Mouvement du bras dans le rep�re local
BC1=Rx(gamma1)*Ry(gamma2)*Rz(gamma3)*lm;
% Mouvement du bras dans le rep�re global
BC=Rx(alpha1)*Ry(alpha2)*Rz(alpha3)*Rx(betha1)*BC1;

% Position bout de la main
OC = OA+AB+BC;

%% r�solution
% on part d'une position de d�part, avec des valeurs connues pour les
% angles et la longueur des bras. On veut atteindre une certainte position
% avec le bout du bras du robot, c'est-�-dire qu'on impose OC.il y a 
% plusieurs mani�res de r�soudre :
% - Boucle vectorielle ferm�e et Newton-Raphson
% - Nessus
% On peut am�liorer la r�solution en l'optimisant :
% - arriver le plus rapidement possible � la position souhait�
% - minimiser la variation d'angle pour chaque articulation <=> minimum de
% mouvements


%% Valeur num�rique

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