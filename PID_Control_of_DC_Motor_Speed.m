%% DC Motor Speed Control using PID
% Simulates a PID speed controller for a DC motor and compares an initial
% and a tuned set of gains, to show how PID gains affect rise time,
% overshoot, settling time and steady-state error in a feedback system.
%
% Requires: MATLAB with the Control System Toolbox.

clc;
clear;
close all;

%% DC motor parameters (assumed values)
R = 2;        % Armature resistance (Ohm)
L = 0.5;      % Armature inductance (H)
K = 0.05;     % Motor constant (V*s/rad = N*m/A)
J = 0.02;     % Rotor inertia (kg*m^2)
b = 0.001;    % Viscous friction coefficient (N*m*s/rad)

%% DC motor transfer function
% Angular speed / armature voltage:
%   w(s)/V(s) = K / ((J*s + b)(L*s + R) + K^2)
num = K;
den = [L*J, (L*b + R*J), (R*b + K^2)];
motor = tf(num, den);

disp('DC Motor Transfer Function:');
motor

%% Initial PID controller
% Starting-point gains
Kp1 = 10;
Ki1 = 20;
Kd1 = 0.5;
pid_initial = pid(Kp1, Ki1, Kd1);
sys_initial = feedback(pid_initial * motor, 1);

%% Tuned PID controller
% Gains increased for a faster response
Kp2 = 30;
Ki2 = 50;
Kd2 = 1.2;
pid_tuned = pid(Kp2, Ki2, Kd2);
sys_tuned = feedback(pid_tuned * motor, 1);

%% Step response comparison
% The open-loop motor settles at dcgain(motor) rad/s per volt (about 11),
% so it is scaled to unit DC gain. All three curves then share the same
% 1 rad/s target and can be compared on one plot.
t = 0:0.01:5;
motor_normalized = motor / dcgain(motor);

[y_ol, t_ol]       = step(motor_normalized, t);
[y_init, t_init]   = step(sys_initial, t);
[y_tuned, t_tuned] = step(sys_tuned, t);

figure;
hold on;
plot(t_ol, y_ol, '--', 'LineWidth', 1.2, 'DisplayName', 'Open-Loop (normalized)');
plot(t_init, y_init, 'LineWidth', 1.5, 'DisplayName', 'Initial PID');
plot(t_tuned, y_tuned, 'LineWidth', 1.5, 'DisplayName', 'Tuned PID');
plot([t(1) t(end)], [1 1], 'k:', 'DisplayName', 'Reference');
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('DC Motor Speed Control Using PID');
legend('Location', 'southeast');
hold off;

%% Performance analysis
info_init  = stepinfo(sys_initial);
info_tuned = stepinfo(sys_tuned);
ss_error_init  = abs(1 - y_init(end));
ss_error_tuned = abs(1 - y_tuned(end));

fprintf('\n%-22s %12s %12s\n', 'Metric', 'Initial PID', 'Tuned PID');
fprintf('%-22s %12.3f %12.3f\n', 'Rise Time (s)',      info_init.RiseTime,     info_tuned.RiseTime);
fprintf('%-22s %12.3f %12.3f\n', 'Settling Time (s)',  info_init.SettlingTime, info_tuned.SettlingTime);
fprintf('%-22s %12.2f %12.2f\n', 'Overshoot (%)',      info_init.Overshoot,    info_tuned.Overshoot);
fprintf('%-22s %12.4f %12.4f\n', 'Steady-State Error', ss_error_init,          ss_error_tuned);
