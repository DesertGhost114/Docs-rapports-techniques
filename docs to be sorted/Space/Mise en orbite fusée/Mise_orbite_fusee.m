

clc
clear
close all

%% données

% masse
m_fusee=150; % [kg]
% altitude position de référence
altitude_z0=0;

% masse volumique air : dépend de la température, qui dépend de l'altitude
% donc à adapter
rho_air=1.2; % [kg/m^2]

% pas du temps
pas_temps=0.1; % [s]

% k : coefficient caractéristique de la géométrie du solide
k=0.5;
% viscosité cinématique
% eta : coefficient de viscosité du fluide (dépend de la température)
eta=0.1;

%% effect de la température
% selon l'altitude, la pression et la température varient, ce qui fait que
% certaines propriétés changent aussi. Retouver la formule du cours TAero
% qui donne température et pression selon l'altitude. utiliser cette valeur
% pour corriger les propriétés à fur à mesure, juqu'à atteindre l'espace. 

%% 
% coefficient de traînée caractérisant la géométrie du solide
Cx=0.5;
% surface à l'air
r=0.2;
S=pi*r^2;

% viscosité
mu=0.1;
% longueur représentative de la fusée 
L=1.2;
% si on stipule que la fusée est plus grande et qu'elle perde des étages à
% certaines étapes du vol, L va changer, ainsi que la masse du carburant. 
%% Force poussée
Fpoussee=2000; % [N]

%% situation initiale
t=0;
z=[];
z(1)=0;
v_fusee=[];
v_fusee(1)=0;
a=[];

%% nombre Reynolds

Re=rho_air*L*v_fusee(1)/mu;

%% calcul
for i=1:100
% accélération terrestre
G=6.667E-11; % N [kg^-2 m^2]
m_terre=5.972E24; % [kg]
rayon_terre=6371000; % [m]
g(i)=G*m_terre/(rayon_terre+altitude_z0+z(i))^2; % [m/s^2] à adapter selon l'altitude
% force pesanteur
Fg(i)=m_fusee*g(i);
% g=9.81; % [m/s^2]

% frottement laminaire
Ffl(i)=-k*eta*v_fusee(i);
% frotement turbulant
Fft(i)=-1/2*rho_air*Cx*S*v_fusee(i)^2;
% conservation quantité de mouvement
% P_fusee=m_fusee*v_fusee;
% P_gaz=m_gaz*v_gaz;
% P_fusee=P_gaz
% équilibre des forces
% dp/dt=somme Force ext=m_fusee*a_fusse+ d m_fusee/dt+v_fusee
% dp/dt=m_gaz*a_gaz+d m_gaz/dt*v_gaz

if Re<2300
    Somme_Force(i)=Fpoussee-Fg(i)-Ffl(i);
else
    Somme_Force(i)=Fpoussee-Fg(i)-Fft(i);
end

% accélération
a(i)=Somme_Force(i)/m_fusee;
%% 
t(i+1)=t(i)+pas_temps;
% vitesse
v_fusee(i+1)=a(i)*(t(i+1)-t(i))+v_fusee(i);
% position 
z(i+1)=a(i)*(t(i+1)-t(i))^2+v_fusee(i+1)*(t(i+1)-t(i))+z(i);

if z(i+1)<0
    z(i+1)=0;
end

Re(i)=rho_air*L*v_fusee(i)/mu;
end


%% Results
for i=1:length(a)
ta(i)=t(i);
end

figure('Name','Cinématique de la fusée au décollage')
subplot(3,1,1);
plot(t,z)
legend('Position')
xlabel('Time [s]')
ylabel('Position [m]')
title('Position de la fusée')

subplot(3,1,2);
plot(t,v_fusee)
legend('vitesse')
xlabel('Time [s]')
title('Vitesse de la fusée')
ylabel('Vitesse [m/s]')

subplot(3,1,3);
plot(ta,a)
title('Accélération de la fusée')
legend('Accéléaration')
xlabel('Time [s]')
ylabel('Accélération [m/s^2]')