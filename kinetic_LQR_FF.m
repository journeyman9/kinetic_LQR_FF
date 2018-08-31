%% Kinetic LQR + feedforward following what HM FSD 2017 did
% Journey McDowell (c) 2018

clear; close all; clc;

%% Parameters
m = 1573;
J = 2873; 
lv = 1.1;%[m]
lh = 1.58; %[m]
cv = 80000; 
ch = 80000;
is = 1; % steering ratio

v = 4.5; %[m/s]

L = lv+lh;
v_ch = sqrt((L.^2)*(cv*ch) / (m*(ch*lh - cv*lv)));
K_k = 1 / (L*(1+(v/v_ch)));

%% Linearized State Space
a11 = -(cv + ch) / (m*v);
a12 = (m*v.^2 - (ch*lh-cv*lv)) / (m*v.^2);
a21 = -(ch*lh - cv*lv) / J;
a22 = -(ch*lh.^2 + cv*lv.^2) / (J*v);

b1 = -(cv / (m*v*is));
b2 = (cv*lv) / (J*is);

A = [a11 a12 0 0;
     a21 a22 0 0;
     0    1  0 0;
     v    0  v 0]; 

 B = [b1;
      b2;
      0;
      0];
 
C = eye(4);

D = zeros(4, 1);

sys = ss(A, B, C, D);

%% Check controllability
controllability  = rank(ctrb(A, B));

%% LQR gains
G = eye(4);
H = D;
Q = [0 0 0 0;
     0 0 0 0;
     0 0 1 0;
     0 0 0 10];
p = 1;
R = 1;

QQ = G'*Q*G;
RR = H'*Q*H + p*R;
NN = G'*Q*H;

[K S e] = lqr(sys, QQ, RR, NN);

%% Feedforward
track_vector = csvread('t_lanechange.txt');
s = track_vector(:, 5);
t = s/v;
curv = [t track_vector(:, 3)];
psi_r = [t track_vector(:, 4)];
y_r = [t track_vector(:, 2)];

Beta_r = [t 0*track_vector(:, 4)];
psi_r_d = [t v*track_vector(:, 3)];

sim_time = t(end, 1);

%% Simulink
y_IC = 0;
% x = [Beta; psi_d; psi; d]
ICs = [deg2rad(0) deg2rad(0) deg2rad(0) y_IC];
vehicleIC = [track_vector(1,1)-y_IC*sin(ICs(2)) track_vector(1,2)+y_IC*cos(ICs(2))];

sim('kinetic_car_sim.slx')

Beta = state(:, 1);
psid = state(:, 2);
psi = state(:, 3);
d = state(:, 3);

%% Plots

figure
subplot 411
plot(tout, rad2deg(Beta))
hold on
plot(tout, des(:, 1), '--r')
ylabel('\beta [{\circ}]')
hold off

subplot 412
plot(tout, rad2deg(psid))
hold on
plot(tout, rad2deg(des(:, 2)), '--r')
hold off
ylabel('\psi_{dot} [{\circ}]')

subplot 413
plot(tout, rad2deg(psi))
hold on
plot(tout, rad2deg(des(:, 3)), '--r')
ylabel('\psi [{\circ}]')

subplot 414
plot(tout, d)
hold on
plot(tout, rad2deg(des(:, 4)), '--r')
ylabel('d [m]')

xlabel('time[s]')
legend('response', 'desired')
movegui('west')

figure
plot(track_vector(:, 1), track_vector(:, 2), '--r')
hold on
plot(odometry(:, 1), odometry(:, 2), 'b')
plot(odometry(1, 1), odometry(1, 2), 'ob')
plot(odometry(end, 1), odometry(end, 2), 'xb')
axis square
axis equal
xlabel('Position in x [m]')
ylabel('Posiiton in y [m]')
legend('desired path', 'vehicle path')
hold off