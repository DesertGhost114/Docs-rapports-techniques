clc
clear
close all

%% Inputs

% Puissance max du moteur
Pm_max = 6000; % [W]
% masse totale du véhicule
m = 415; % [kg]
% rapport de réduction
R = 100; % [-]
% pente
slope = 48.4; % [%]
% max : 48.4%
% dimensions du véhicule (ici Estrima Biro)
L = 1.74; % Longueur en [m]
l = 1.03; % Largeur en [m]
h = 1.565; % Hauteur en [m]
% Taille des pneux
% exemple : 175/50 R13
p1 = 175; % [mm]
p2 = 50; % [%]
p3 = 13; % [pouces]

%% Constantes

g = 9.81;
% écoulement laminaire
k = 5;
eta = 18.5E-5;
% écoulement turbulant
Cx = 1.05; % hypothèse d'un cube
rho = 1.2; %
S = l*h;
% Reynolds
nu = 1.516E-5;
mu = 1.85E-4;

%% Calculs 
% angle de la pente
angle = atan(slope/100); % [rad]
angle_deg = rad2deg(angle); % [°]
% rayon de la roue
rayon = (p1*p2/100+25.4*p3/2)/1000; % [m]

%% Situation d'arrêt en pente maximale
ax = 0;
v = 0;
x = 0;

% Force pesanteur
Fp = m*g;
% force de frottement aérodynamique
Re = v*max([L,l,h])/nu;
% laminaire
Ffl = k*eta*v;
% turbulant
Fft = 0.5*rho*Cx*S*v^2;
if Re < 3000
    Ff = Ffl;
else
    Ff = Fft;
end
% Equilibre des forces
% Fx = Fy = 0
% Fx = -Fp*sin(angle) + Fr -Ff = m*ax = 0
% Forces sur les roues
Fr = m*ax+Fp*sin(angle) + Ff;
% Couple sur les roues pour maintenir le véhicule en place. 
Cr = Fr*rayon;
% Fy = -Fp*cos(angle) + Fn = 0
Fn = Fp*cos(angle);

%% accélérer et atteindre une vitesse constante de 25km/h
vf = 25/3.6; % [m/s]
% on augemente la force sur les roues pour déplacer le véhicule
Fr = Fr + 250;
t = 0;
% on itère par pas de 0.1s
deltat= 0.1;

a_vec = [];
v_vec = [];
v_vec_kmh = [];
x_vec = [];
t_vec = [];

Ff_vec = [];
Fr_vec = [];
Proue = 0;
% on suppose qu'il n'y pas de pertes au palier : Pm = Proue
Proue_vec = [];
while v < vf && Proue < Pm_max
    % Equilibre des forces
    % Fx = -Fp*sin(angle) + Fr -Ff = m*ax
    % nouvelle accélération
    ax = (-Fp*sin(angle) + Fr -Ff)/m;
    a_vec = [a_vec,ax];
    x = 0.5*ax*(deltat)^2+v*deltat+x;
    x_vec = [x_vec,x];
    v = ax * deltat+ v;
    v_vec = [v_vec,v];
    v_vec_kmh = [v_vec_kmh, v*3.6];
    Re = v*max([L,l,h])/nu;
    t = t + deltat;
    t_vec = [t_vec,t];
    % laminaire
    Ffl = k*eta*v;
    % turbulant
    Fft = 0.5*rho*Cx*S*v^2;
    if Re < 3000
        Ff = Ffl;
    else
        Ff = Fft;
    end
    Ff_vec = [Ff_vec,Ff];
    
    % Puissance moteur
    Proue = Fr*v; % [W]
    Proue_vec = [Proue_vec,Proue];
end

figure
subplot(2,2,1)
plot(t_vec,a_vec)
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')
grid on;

subplot(2,2,2)
% plot(t_vec,v_vec)
% xlabel('Time [s]')
% ylabel('Velocity [m/s]')
[hAx,hLine1,hLine2] = plotyy(t_vec,v_vec,t_vec, v_vec_kmh); %plots voltage and current vs time
% title('Voltage and Current vs Time : Cycle 451 to 469c')
xlabel('Time [s]')
ylabel(hAx(1),'Vitesse [m/s]') % left y-axis 
ylabel(hAx(2),'Vitesse [km/h]') % right y-axis
grid on;

subplot(2,2,3)
plot(t_vec,x_vec)
xlabel('Time [s]')
ylabel('Distance [m]')
grid on;

subplot(2,2,4)
plot(t_vec,Proue_vec)
xlabel('Time [s]')
ylabel('Puissance sur les roues [W]')
grid on;
print('R','-dpng') % save the figure as png file
print('R','-djpeg')
savefig('R.fig')

% figure('Name','Vitesse'); % new figure
% [hAx,hLine1,hLine2] = plotyy(t_vec,v_vec,t_vec, v_vec_kmh); %plots voltage and current vs time
% % title('Voltage and Current vs Time : Cycle 451 to 469c')
% xlabel('Time [s]')
% ylabel(hAx(1),'Vitesse [m/s]') % left y-axis 
% ylabel(hAx(2),'Vitesse [km/h]') % right y-axis
% grid on;

disp(['Avec un moteur électrique de ', num2str(Pm_max),' W, la vitesse maximale du véhicule est de ', num2str(max(round(v_vec_kmh,1))), ' km/h avec une pente de ',num2str(slope), '%.'])

% msg = sprintf('Avec un moteur électrique de %d W, la vitesse maximale du véhicule est de %d km/h avec une pente de %d pourcent.', Pm_max, max(v_vec_kmh),slope);
% disp(msg)

% figure('Name','Cycle 451 to 500'); % new figure
% [hAx,hLine1,hLine2] = plotyy(time,voltage,time,current); %plots voltage and current vs time
% title('Voltage and Current vs Time : Cycle 451 to 469c')
% xlabel('Time [s]')
% ylabel(hAx(1),'Voltage [V]') % left y-axis 
% ylabel(hAx(2),'Current [A]') % right y-axis
% grid on;
% print('V_I_vs_time_cycle_451_469c','-dpng') % save the figure as png file
% print('V_I_vs_time_cycle_451_469c','-djpeg')
% savefig('V_I_vs_time_cycle_451_469c.fig')