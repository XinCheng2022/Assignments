close all; clc
L1 = 4.15
L2 = 8.3
b = 0.35   
v1r = 100 % m/s
max_steering =  deg2rad(30)
delta = linspace(-max_steering, max_steering, 1000 );
%% vp error
vp = v1r * ((b * tan(delta) / L1) .^ 2 +1 ) .^0.5;
vp2 = v1r * ((b * (delta) / L1) .^ 2 +1 ) .^0.5;

% figure
% plot(delta, vp)
% hold on 
% plot( delta, vp2)
figure
plot(rad2deg(delta), vp-vp2); grid on
xlabel('steering angle, degree')
ylabel('speed error at 100m/s, m/s')
title('replace tan(delta) with delta when computing vp')
%% theta1dot error
theta1dot = v1r * tan(delta) / L1;
theta1dot2 = v1r * delta / L1;
figure
plot(rad2deg(delta), theta1dot-theta1dot2)
xlabel('steering angle, degree')
ylabel('v=100m/s, theta1dot error, rad')
grid on
title('replace tan(delta) with delta when computing theta1dot')
%% betaP error
betaP = atan(b * tan(delta) / L1);
betaP2 = atan(b * delta / L1);
figure
plot(rad2deg(delta), betaP-betaP2)
xlabel('steering angle, degree')
ylabel('v=100m/s, betaP error, rad')
title('replace tan(delta) with delta when computing betaP')
grid on
% figure
% plot(rad2deg(delta), rad2deg(betaP))
% title('betaP')

figure
plot(rad2deg(delta), tan(betaP)-sin(betaP), 'DisplayName','tan(betaP)-sin(betaP)');hold on
plot(rad2deg(delta), cos(betaP) - 1,'DisplayName','cos(betaP) -1')
grid on
title('betaP')
legend

sinbetaP = b * tan(delta) / L1
sinbetaP2 = b * delta / L1
figure
plot(rad2deg(delta), sinbetaP-sinbetaP2);grid on
title(' sinbetaP(tan delta)-sinbetaP2( delta)')
%% theta2dot error

A1 = b * delta / L1;
A2 = ((b * delta / L1) .^2 +1) .^ 0.5 ;
theta = deg2rad(10)
theta2dot2 = (v1r / L1) * (sin(theta) + cos(theta) * b* delta/L1);
theta2dot = (sin(theta) .* cos(betaP) + cos(theta) .* sin(betaP)) .* vp / L1;
figure
plot(rad2deg(delta),theta2dot - theta2dot2)
title('simplified theta2dot error, assume theta is 15deg, max steering 15deg')
xlabel('steering angle, degree')
ylabel('theta2dot error, rad/s')
grid on

%%


