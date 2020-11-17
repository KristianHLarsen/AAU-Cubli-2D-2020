clc
clear all
close all

data = csvread('daisy_test.csv');

yyaxis left
xlabel('t [ms]')
plot(data(:,1), data(:,7), 'Linewidth', 2)
ylabel('\theta_f [rad]') 
hold on
yyaxis right
ylabel('\tau_m [Nm]')
ylim([-0.6 0.8])
plot(data(:,1), data(:,6), '-.','Linewidth', 1.2)
grid on
set(gca,'XLim',[0 2150])
set(gca,'XTick',(0:500:2150))




title('Angular position of frame', 'FontSize', 10);
legend('Position of frame', 'Motor torque')