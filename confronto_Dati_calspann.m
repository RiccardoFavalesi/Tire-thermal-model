clc
close all
clear all

data9 = load("Dati Calspan\B2356raw9.mat")
figure(1)
subplot(2,1,1)
plot(data9.RST)
hold on
plot(data9.TSTC)
plot(data9.AMBTMP)
%%
data8 = load("B2356raw8.mat");
subplot(2,1,2)
plot(data8.RST)
hold on
plot(data8.TSTC)
plot(data8.AMBTMP)

linkaxes([subplot(2,1,1), subplot(2,1,2)], 'x');