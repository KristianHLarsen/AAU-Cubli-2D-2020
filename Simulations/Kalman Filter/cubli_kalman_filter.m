clc
close all
clear all
%% Load file
% 1. Timestamp - 2. Potentiometer angle - 3. Gyro speed of frame - 4. Acceleration in X - 5. Acceleration in Y - 6. Speed of wheel
data = dlmread('balance_data_3.csv', ';');

time = data(:,1);
time_start = time(1);
time = time-time_start;
%1 Time - 2 Potentiometer - 3 Speed frame - 4 AcX - 5 AcY 
% 6 SPW - 7 Torque - 8 Complementary angle 
% 9 KF angle - 10 KF frame speed - 11 KF wheel speed - 12 time for KF

%% measurements
% 1. Angle of frame - 2. Gyro speed of frame - 3. Speed of wheel
% z = transpose([-(atan2(data(:,4), data(:,5)) - pi/4) , data(:,3), 2*data(:,6)]);
z = transpose([-(atan2(-data(:,5), -data(:,4)) + pi - pi/4) , data(:,3), 2*data(:,6)]);
pot = data(:,2)*(pi)/180;
filtered_ang_pos = data(:,7);
% plot(z(:,1))
% hold on
% plot(data(:,2)*pi/180)
%% Model parameter definitions

g = 9.816; % Gravitational acceleration [m/s^2]
m_w = 0.220; % Mass of the wheel [kg]
l_w = 0.092; % Distance between flywheel's COM and origin [m]
J_w = 0.699*10^(-3); % Inertia of the flyweheel [kg*m^2]
b_w = 1.93*10^(-5); % Damping coefficient for the flywheel [N*m*s/rad]
m_f = 0.766-m_w; % Mass of the frame [kg]
l_f = 0.0958; % Distance between frame COM and the origin [m]
J_f = 0.0067; % Inertia of the frame [kg*m^2]
b_f = (0.0036+0.0033+0.0038)/3; % Damping coefficient for the frame [N*m*s/rad]
%% State space model (continuous)

A = zeros(3,3);
A(1,1) = 0;
A(1,2) = 1;
A(1,3) = 0;
A(2,1) = g*(m_f*l_f+m_w*l_w)/(J_f + m_w*(l_w)^2);
A(2,2) = -b_f/(J_f+m_w*(l_w)^2);
A(2,3) = b_w/(J_f+m_w*(l_w)^2);
A(3,1) = - g*(m_f*l_f+m_w*l_w)/(J_f+m_w*(l_w)^2);
A(3,2) = b_f/(J_f+m_w*(l_w)^2);
A(3,3) = -b_w*(J_w+J_f+m_w*(l_w)^2)/(J_w*(J_f+m_w*(l_w)^2));

B = zeros(3,1);
B(1) = 0;
B(2) = -1/(J_f+m_w*(l_w)^2);
B(3) = (J_w+J_f+m_w*(l_w)^2)/(J_w*(J_f+m_w*(l_w)^2));

C = eye(3);
D = zeros(3,1);
sys = ss(A,B,C,D);

u = data(:,7); %% Input vector

%% Discrete State space
dt = 0.002; %% sampling time
d_sys = c2d(sys,dt);

A_d = d_sys.A;
B_d = d_sys.B;


%% %%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%
% initial guess of the state
x_prior(:,1) = [0 0 0];
x_post(:,1) = [0 0 0];

P_prior = eye(3);
P_post = eye(3);

% Mapping between states and measurements
H = eye(3);

%%% Covariance of the noise in the measurement
R = [0.01 0 0;
        0 1 0;
        0 0 1000];

%%% Covariance of the noise in the process
Q = [0.00001 0 0;
        0 1 0;
        0 0 50];

% Main Kalman loop
for i = 2:length(u)
    x_prior(:,i) = A_d*x_post(:,i-1) + B_d*u(i);
    P_prior = A_d*P_post*transpose(A_d) + Q;
    K = P_prior * transpose(H) * ...
           inv((H * P_prior * transpose(H) + R));    
    x_post(:,i) = x_prior(:,i) + K*(z(:,i) - H*x_prior(:,i));
    P_post = P_prior - K*H*P_prior;
end

Kalman_poles = eig(A_d - H*K);

%% %%%%%%% Plotting results %%%%%%%%%%%%%
figure(1)
subplot(3, 1, 1)
plot(filtered_ang_pos, 'Linewidth', 0.5)
hold on
plot(x_post(1,:), 'Linewidth', 2)
hold on
plot(pot, 'Linewidth', 2)
xlabel('time [ms]') 
ylabel('\theta_F [rad]') 
legend('Complementary filter', 'Kalman filter', 'Raw potentiometer data') %% Add if plot(filtered_ang_pos) is enabled
% legend('Kalman filter', 'Raw potentiometer data') %% Remove this if plot(filtered_ang_pos) is enabled
title('Angular position of frame', 'FontSize', 10);



subplot(3, 1, 2)
plot(x_post(2,:), 'Linewidth', 1)
hold on
plot (z(2,:))
xlabel('time [ms]') 
ylabel('\omega_F [rad/s]') 
legend('Kalman filter', 'Raw gyro data') %% Add if plot(filtered_ang_pos) is enabled
title('Angular velocity of frame', 'FontSize', 10);

 
subplot(3, 1, 3)
plot(x_post(3,:), 'Linewidth', 1)
hold on
plot (z(3,:))
xlabel('time [ms]') 
ylabel('\omega_w [rad]') 
legend('Kalman filter', 'Raw escon data') %% Add if plot(filtered_ang_pos) is enabled
title('Angular velocity of reaction wheel', 'FontSize', 10);
 
%% %%%%%%% Plotting partial results %%%%%%%%%%%%%
t1 = 1401;
t2 = 1700;

figure(2)
subplot(3, 1, 1)
plot(time(t1:t2), filtered_ang_pos(t1:t2), 'Linewidth', 0.5)
hold on
plot(x_post(1,(t1:t2)), 'Linewidth', 2)
hold on
plot(pot(t1:t2), 'Linewidth', 2)
xlabel('time [ms]') 
ylabel('\theta_F [rad]') 
legend('Complementary filter', 'Kalman filter', 'Raw potentiometer data', 'Location','southwest') %% Add if plot(filtered_ang_pos) is enabled
% legend('Kalman filter', 'Raw potentiometer data') %% Remove this if plot(filtered_ang_pos) is enabled
title('Angular position of frame', 'FontSize', 10);



subplot(3, 1, 2)
plot(x_post(2,(t1:t2)), 'Linewidth', 1)
hold on
plot (z(2,(t1:t2)))
xlabel('time [ms]') 
ylabel('\omega_F [rad/s]') 
legend('Kalman filter', 'Raw gyroscope data', 'Location','southwest') %% Add if plot(filtered_ang_pos) is enabled
title('Angular velocity of frame', 'FontSize', 10);

 
subplot(3, 1, 3)
plot(x_post(3,(t1:t2)), 'Linewidth', 1)
hold on
plot (z(3,(t1:t2)))
xlabel('time [ms]') 
ylabel('\omega_w [rad]') 
legend('Kalman filter', 'Raw escon driver data', 'Location','southwest') %% Add if plot(filtered_ang_pos) is enabled
title('Angular velocity of reaction wheel', 'FontSize', 10);
 
