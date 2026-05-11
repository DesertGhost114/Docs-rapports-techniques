close all

load T_rpm.mat

rpm_nom = 1160; % [rpm], at 100%

T_nom = 30.3; % [Nm], at 1 pu

rpm = T_rpm(:,1); % rpm in %
T = T_rpm(:,2); % Torque in pu

rpm_val = rpm*rpm_nom/100;% [rpm]
T_val = T*T_nom;% [Nm]

omega = rpm_val * 2*pi/60;
power = T_val.*omega;

figure
subplot(2,2,1)
plot(rpm_val, T_val)
xlabel('Speed [rpm]')
ylabel('Torque [Nm]')
grid on
subplot(2,2,2)
plot(omega, T_val)
xlabel('Speed [rad/s]')
ylabel('Torque [Nm]')
grid on
subplot(2,2,3)
plot(omega, power)
xlabel('Speed [rad/s]')
ylabel('Power [W]')
grid on

T_fit=fit(omega,T_val,'poly5');
T_fitted = T_fit.p1*omega.^5+T_fit.p2 *omega.^4+T_fit.p3*omega.^3+T_fit.p4*omega.^2+T_fit.p5*omega.^+T_fit.p6;



figure
plot(omega,T_fitted,omega, T_val)
legend('T fit','T data')

