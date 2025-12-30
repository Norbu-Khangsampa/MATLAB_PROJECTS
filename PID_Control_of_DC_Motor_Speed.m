%% DC Motor Speed Control using PID
% Simple simulation of a PID controller for DC motor speed control
% This was done to understand how PID affects rise time, overshoot,
% and steady-state error in a feedback system.

clc;
clear;
close all;

%% DC motor parameters (assumed values)
R = 2;        % Resistance (Ohms)
L = 0.5;      % Inductance (H)
K = 0.05;     % Motor constant
J = 0.02;     % Inertia
b = 0.001;    % Friction coefficient

%% DC motor transfer function
% Speed / Voltage transfer function
num = K;
den = [L*J (L*b + R*J) (R*b + K^2)];
motor = tf(num, den);

disp('DC Motor Transfer Function:');
motor

%% Initial PID controller
% These values were chosen as a starting point
Kp1 = 10;
Ki1 = 20;
Kd1 = 0.5;
pid_initial = pid(Kp1, Ki1, Kd1);
% Closed-loop system with initial PID
sys_initial = feedback(pid_initial * motor, 1);

%% Tuned PID controller
% Gains were increased to improve response
Kp2 = 30;
Ki2 = 50;
Kd2 = 1.2;
pid_tuned = pid(Kp2, Ki2, Kd2);
% Closed-loop system after tuning
sys_tuned = feedback(pid_tuned * motor, 1);

%% Step response comparison
t = 0:0.01:5;
figure;
hold on;
% Open-loop response
[y_ol, t_ol] = step(motor, t);
plot(t_ol, y_ol, '--', 'DisplayName', 'Open-Loop');
% Initial PID response
[y_init, t_init] = step(sys_initial, t);
plot(t_init, y_init, 'DisplayName', 'Initial PID');
% Tuned PID response
[y_tuned, t_tuned] = step(sys_tuned, t);
plot(t_tuned, y_tuned, 'DisplayName', 'Tuned PID');
% Reference line
plot([0 5], [1 1], ':', 'DisplayName', 'Reference');
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('DC Motor Speed Control Using PID');
legend;
hold off;

%% Performance analysis
% Checking how the response changes after tuning
info_init = stepinfo(sys_initial);
info_tuned = stepinfo(sys_tuned);
ss_error_init = abs(1 - y_init(end));
ss_error_tuned = abs(1 - y_tuned(end));
fprintf('\nInitial PID:\n');
fprintf('Settling Time: %.2f s\n', info_init.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info_init.Overshoot);
fprintf('Steady-State Error: %.4f\n', ss_error_init);
fprintf('\nTuned PID:\n');
fprintf('Settling Time: %.2f s\n', info_tuned.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info_tuned.Overshoot);
fprintf('Steady-State Error: %.4f\n', ss_error_tuned);