

clc
clear
close all

%% données
% longueur bras 
% l1 : bras 1 
% l2 : bras 2
% l3 : bras 3
% l4 : pince
syms l1 alpha1 l2 beta1 l3 gamma1 l4 delta1 delta2 delta3

%% Matrices de roations
% matrice de rotation (en faire des fonctions)
% Rotation selon axe X : Rotx
% Rotation selon axe Y : Roty
% Rotation selon axe Z : Rotz
%% Repère global et fixe au point 0 : la base
% R={x,y,z}

% Repère mobile 1 : rotation du bras 1 selon z
% R1={x1,y1,z2}
% Vecteur du bras : OA
OA_R1=transpose([0,0,l1]);
OA_R=Rotz(alpha1)*OA_R1;

% Repère mobile 2 : rotation du bras 2 selon x
% R2={x2,y2,z2}
% Vecteur du bras : AB
AB_R2=transpose([0,l2,0]);
AB_R1=Rotx(beta1)*AB_R2;
AB_R=Rotz(alpha1)*AB_R1;

% Repère mobile 3 : rotation du bras 3 selon y
% R3={x3,y3,z3}
% Vecteur du bras : BC
BC_R3=transpose([l3,0,0]);
BC_R2=Roty(gamma1)*BC_R3;
BC_R1=Rotx(beta1)*BC_R2;
BC_R=Rotz(alpha1)*BC_R1;

% Repère mobile 3 : rotation de la pince selon y
% R4={x4,y4,z4}
% Vecteur du bras : CD
CD_R4=transpose([0,0,-l4]);
CD_R3=Rotx(delta1)*Roty(delta2)*Rotz(delta3)*CD_R4;
CD_R2=Roty(gamma1)*CD_R3;
CD_R1=Rotx(beta1)*CD_R2;
CD_R=Roty(alpha1)*CD_R1;

% position de la pince selon le repère R
OB_R=OA_R+AB_R;
OC_R=OA_R+AB_R+BC_R;
OD_R=OA_R+AB_R+BC_R+CD_R;

%% position initial
% 
l1=1;
alpha1=0;

%
l2=1;
beta1=0;
% 
l3=.2;
gamma1=0;
% 
l4=.2;
delta1=0;
delta2=0;
delta3=0;

O=[0,0,0]';
OA_R0=eval(OA_R);
OB_R0=eval(OB_R);
OC_R0=eval(OC_R);
OD_R0=eval(OD_R);
% Positions=horzcat(O,OA_R0,OB_R0,OC_R0,OD_R0);
OA_vec=[linspace(O(1),OA_R0(1));
    linspace(O(2),OA_R0(2));
    linspace(O(3),OA_R0(3))];

AB_vec=[linspace(OA_R0(1),OB_R0(1));
    linspace(OA_R0(2),OB_R0(2));
    linspace(OA_R0(3),OB_R0(3))];

figure
plot3(0,0,0,'-o',OA_R0(1),OA_R0(2),OA_R0(3),'-x',OB_R0(1),OB_R0(2),OB_R0(3),'-s')
plot3(OA_vec(1,:),OA_vec(2,:),OA_vec(3,:),AB_vec(1,:),AB_vec(2,:),AB_vec(3,:))
for i=1:N


end

%% Results
% for i=1:length(a)
% ta(i)=t(i);
% end
% 
% figure('Name','Cinématique de la fusée au décollage')
% subplot(3,1,1);
% plot(t,z)
% legend('Position')
% xlabel('Time [s]')
% ylabel('Position [m]')
% title('Position de la fusée')
% 
% subplot(3,1,2);
% plot(t,v_fusee)
% legend('vitesse')
% xlabel('Time [s]')
% title('Vitesse de la fusée')
% ylabel('Vitesse [m/s]')
% 
% subplot(3,1,3);
% plot(ta,a)
% title('Accélération de la fusée')
% legend('Accéléaration')
% xlabel('Time [s]')
% ylabel('Accélération [m/s^2]')