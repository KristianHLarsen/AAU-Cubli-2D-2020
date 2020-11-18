clear all
close all
clc

load('databuffered.mat')
plot(C)
hold on
plot(WB1)
plot(val1)
plot(D)
plot(B)
grid on
xlabel('Packet number')
xlabel('Packet number', 'Fontsize',20)
ylabel('Delay roundtrip (us)', 'Fontsize',20)