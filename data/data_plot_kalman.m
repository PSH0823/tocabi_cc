%%
clc
clear all
close all

set(0, 'DefaultAxesFontSize', 20)

data = readmatrix('kalman_filter_log.txt');

pos = data(:, 1:3);
vel = data(:, 4:6);

dt = 0.0005;
t = (1:size(data,1)) * dt;   % time index (or replace with real time if available)

%% Position plots
figure
sgtitle('Position')

subplot(3,1,1)
plot(t, pos(:,1))
ylabel('x')
% grid on

subplot(3,1,2)
plot(t, pos(:,2))
ylabel('y')
% grid on

subplot(3,1,3)
plot(t, pos(:,3))
ylabel('z')
xlabel('Time step')
% grid on

%% Velocity plots
figure
sgtitle('Velocity')

subplot(3,1,1)
plot(t, vel(:,1))
ylabel('v_x')
% grid on

subplot(3,1,2)
plot(t, vel(:,2))
ylabel('v_y')
% grid on

subplot(3,1,3)
plot(t, vel(:,3))
ylabel('v_z')
xlabel('Time step')
% grid on

%% Position comparison with sensor measurement
pos_sensor = data(:, 7:9);

figure
sgtitle('Raw Position')
subplot(3,1,1)
plot(t, pos_sensor(:,1))
ylabel('x_{sensor}')
% grid on

subplot(3,1,2)
plot(t, pos_sensor(:,2))
ylabel('y_{sensor}')
% grid on

subplot(3,1,3)
plot(t, pos_sensor(:,3))
ylabel('z_{sensor}')
xlabel('Time step')
% grid on

%% Validating the data
% A = [1 dt; 0 1];
% R = 1;
% C = [1 0];
% Q = [0.01 0; 0 0.1];
% P = [1 0; 0 1];
% q_pred = [1.68321; 0];
% q_est = [1.68321; 0];
% 
% for i = 1 : 1 : 10
%     q_pred = A * q_est;
%     P = A * P * A' + Q;
% 
%     K = P * C' * (C* P * C' + R)^(-1);
%     q_est = q_pred + K * (y - C * q_pred);
%     P = (ones(2,2)- K * C) * P;
%     
%     fprintf('%dth q_est: %f',i, q_est)
% end
