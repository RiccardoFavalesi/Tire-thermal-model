%% Intro
clear
close all
clc

load('Final_pop30.mat');

nbins = 20;

tw = 1270;
wb = 1530;

%% lower bounds
xl       = [    25, ... %RCHf
                tw, ... %FVSAf
                 8, ... %aKingPinf
               -20, ... %SRf
               305, ... %zUBJf
               155, ... %zLBJf
                 6, ... %aCasterf
                10, ... %CTf
              wb/2, ... %SVSAf
               4.2, ... %fif
             wb+50, ... %xTROPf
               145, ... %zTROPf
                10, ... %betaf
               -30, ... %theta_F_UCAf
                15, ... %theta_R_UCAf
               -30, ... %theta_F_LCAf
                15, ... %theta_R_LCAf
                50, ... %RCHr
                tw, ... %FVSAr
                10, ... %aKingPinr
               -20, ... %SRr
               295, ... %zUBJr
               155, ... %zLBJr
                 2, ... %aCasterr
               -50, ... %CTr
              wb/2, ... %SVSAr
               4.2, ... %fir
              -120, ... %xTROPr
           tw/2-80, ... %yTROPr
               220, ... %zTROPr
               -35, ... %theta_F_UCAr
                15, ... %theta_R_UCAr
               -35, ... %theta_F_LCAr
                15, ... %theta_R_LCAr
               0.5, ... %tube_front_A
               0.5, ... %tube_front_B
               0.5, ... %tube_front_C
               0.5, ... %tube_front_D
               0.5, ... %tube_rear_A
               0.5, ... %tube_rear_B
               0.5, ... %tube_rear_C
               0.5, ... %tube_rear_D
               0.5, ... %front_pullrod
               0.5, ... %rear_pushrod
               0.5, ... %front_tierod
               0.5, ... %rear_tierod
               200, ... %motorphi_F
                60, ... %motorphi_R
                 0];    %ack_construction

%% upper bounds
xu       = [    75, ... %RCHf
             10*tw, ... %FVSAf
                17, ... %aKingPinf
                30, ... %SRf
               350, ... %zUBJf
               167, ... %zLBJf
                12, ... %aCasterf
                30, ... %CTf
              4*wb, ... %SVSAf
               8.5, ... %fif
            wb+120, ... %xTROPf
               170, ... %zTROPf
                20, ... %betaf
               -15, ... %theta_F_UCAf
                25, ... %theta_R_UCAf
               -15, ... %theta_F_LCAf
                25, ... %theta_R_LCAf
               125, ... %RCHr
            2.5*tw, ... %FVSAr
                17, ... %aKingPinr
                30, ... %SRr
               350, ... %zUBJr
               177, ... %zLBJr
                12, ... %aCasterr
                10, ... %CTr
              2*wb, ... %SVSAr
               8.5, ... %fir
               -80, ... %xTROPr
           tw/2-35, ... %yTROPr
               250, ... %zTROPr
               -15, ... %theta_F_UCAr
                25, ... %theta_R_UCAr
               -15, ... %theta_F_LCAr
                25, ... %theta_R_LCAr
               3.4, ... %tube_front_A
               3.4, ... %tube_front_B
               3.4, ... %tube_front_C
               3.4, ... %tube_front_D
               3.4, ... %tube_rear_A
               3.4, ... %tube_rear_B
               3.4, ... %tube_rear_C
               3.4, ... %tube_rear_D
               3.4, ... %front_pullrod
               3.4, ... %rear_pushrod
               3.4, ... %front_tierod
               3.4, ... %rear_tierod
               300, ... %motorphi_F
               130, ... %motorphi_R
              0.75];    %ack_construction

%% RCH front

subplot(2,13,1)
edges = linspace(xl(1), xu(1), nbins);
rch_f = zeros(1,length(pop));
for ii = 1:length(pop)
    rch_f(ii) = pop(ii).var(1);
end
histogram(rch_f,edges)
legend('rch_f')
xlim([xl(1), xu(1)])

%% FVSA f

subplot(2,13,2)
edges = linspace(xl(2), xu(2), nbins);
FVSA_f = zeros(1,length(pop));
for ii = 1:length(pop)
    FVSA_f(ii) = pop(ii).var(2);
end
histogram(FVSA_f,edges)
legend('FVSA_f')
xlim([xl(2), xu(2)])

%% Kingpin Angle front

subplot(2,13,3)
edges = linspace(xl(3), xu(3), nbins);
aKingPin_f = zeros(1,length(pop));
for ii = 1:length(pop)
    aKingPin_f(ii) = pop(ii).var(3);
end
histogram(aKingPin_f,edges)
legend('aKingPin_f')
xlim([xl(3), xu(3)])

%% Scrub Radius front

subplot(2,13,4)
edges = linspace(xl(4), xu(4), nbins);
SR_f = zeros(1,length(pop));
for ii = 1:length(pop)
    SR_f(ii) = pop(ii).var(4);
end
histogram(SR_f,edges)
legend('SR_f')
xlim([xl(4), xu(4)])

%% zUBJ front

subplot(2,13,5)
edges = linspace(xl(5), xu(5), nbins);
zUBJ_f = zeros(1,length(pop));
for ii = 1:length(pop)
    zUBJ_f(ii) = pop(ii).var(5);
end
histogram(zUBJ_f,edges)
legend('zUBJ_f')
xlim([xl(5), xu(5)])

%% zLBJ front

subplot(2,13,6)
edges = linspace(xl(6), xu(6), nbins);
zLBJ_f = zeros(1,length(pop));
for ii = 1:length(pop)
    zLBJ_f(ii) = pop(ii).var(6);
end
histogram(zLBJ_f,edges)
legend('zLBJ_f')
xlim([xl(6), xu(6)])

%% Caster Angle front

subplot(2,13,7)
edges = linspace(xl(7), xu(7), nbins);
aCaster_f = zeros(1,length(pop));
for ii = 1:length(pop)
    aCaster_f(ii) = pop(ii).var(7);
end
histogram(aCaster_f,edges)
legend('aCaster_f')
xlim([xl(7), xu(7)])

%% Caster Trail front

subplot(2,13,8)
edges = linspace(xl(8), xu(8), nbins);
CT_f = zeros(1,length(pop));
for ii = 1:length(pop)
    CT_f(ii) = pop(ii).var(8);
end
histogram(CT_f,edges)
legend('CT_f')
xlim([xl(8), xu(8)])

%% SVSA front

subplot(2,13,9)
edges = linspace(xl(9), xu(9), nbins);
SVSA_f = zeros(1,length(pop));
for ii = 1:length(pop)
    SVSA_f(ii) = pop(ii).var(9);
end
histogram(SVSA_f,edges)
legend('SVSA_f')
xlim([xl(9), xu(9)])

%% Phi front

subplot(2,13,10)
edges = linspace(xl(10), xu(10), nbins);
phi_f = zeros(1,length(pop));
for ii = 1:length(pop)
    phi_f(ii) = pop(ii).var(10);
end
histogram(phi_f,edges)
legend('phi_f')
xlim([xl(10), xu(10)])

%% xTROP front

subplot(2,13,11)
edges = linspace(xl(11), xu(11), nbins);
xTRO_f = zeros(1,length(pop));
for ii = 1:length(pop)
    xTRO_f(ii) = pop(ii).var(11);
end
histogram(xTRO_f,edges)
legend('xTRO_f')
xlim([xl(11), xu(11)])

%% zTROP front

subplot(2,13,12)
edges = linspace(xl(12), xu(12), nbins);
zTRO_f = zeros(1,length(pop));
for ii = 1:length(pop)
    zTRO_f(ii) = pop(ii).var(12);
end
histogram(zTRO_f,edges)
legend('zTRO_f')
xlim([xl(12), xu(12)])

%% Beta front

subplot(2,13,13)
edges = linspace(xl(13), xu(13), nbins);
beta_f = zeros(1,length(pop));
for ii = 1:length(pop)
    beta_f(ii) = pop(ii).var(13);
end
histogram(beta_f,edges)
legend('beta_f')
xlim([xl(13), xu(13)])

%% RCH rear

subplot(2,13,14)
edges = linspace(xl(18), xu(18), nbins);
rch_r = zeros(1,length(pop));
for ii = 1:length(pop)
    rch_r(ii) = pop(ii).var(18);
end
histogram(rch_r,edges)
legend('rch_r')
xlim([xl(18), xu(18)])

%% FVSA rear

subplot(2,13,15)
edges = linspace(xl(19), xu(19), nbins);
FVSA_r = zeros(1,length(pop));
for ii = 1:length(pop)
    FVSA_r(ii) = pop(ii).var(19);
end
histogram(FVSA_r,edges)
legend('FVSA_r')
xlim([xl(19), xu(19)])

%% Kingpin Angle rear

subplot(2,13,16)
edges = linspace(xl(20), xu(20), nbins);
aKingPin_r = zeros(1,length(pop));
for ii = 1:length(pop)
    aKingPin_r(ii) = pop(ii).var(20);
end
histogram(aKingPin_r,edges)
legend('aKingPin_r')
xlim([xl(20), xu(20)])

%% Scrub Radius rear

subplot(2,13,17)
edges = linspace(xl(21), xu(21), nbins);
SR_r = zeros(1,length(pop));
for ii = 1:length(pop)
    SR_r(ii) = pop(ii).var(21);
end
histogram(SR_r,edges)
legend('SR_r')
xlim([xl(21), xu(21)])

%% zUBJ rear

subplot(2,13,18)
edges = linspace(xl(22), xu(22), nbins);
zUBJ_r = zeros(1,length(pop));
for ii = 1:length(pop)
    zUBJ_r(ii) = pop(ii).var(22);
end
histogram(zUBJ_r,edges)
legend('zUBJ_r')
xlim([xl(22), xu(22)])

%% zLBJ rear

subplot(2,13,19)
edges = linspace(xl(23), xu(23), nbins);
zLBJ_r = zeros(1,length(pop));
for ii = 1:length(pop)
    zLBJ_r(ii) = pop(ii).var(23);
end
histogram(zLBJ_r,edges)
legend('zLBJ_r')
xlim([xl(23), xu(23)])

%% Caster Angle rear

subplot(2,13,20)
edges = linspace(xl(24), xu(24), nbins);
aCaster_r = zeros(1,length(pop));
for ii = 1:length(pop)
    aCaster_r(ii) = pop(ii).var(24);
end
histogram(aCaster_r,edges)
legend('aCaster_r')
xlim([xl(24), xu(24)])

%% Caster Trail rear

subplot(2,13,21)
edges = linspace(xl(25), xu(25), nbins);
CT_r = zeros(1,length(pop));
for ii = 1:length(pop)
    CT_r(ii) = pop(ii).var(25);
end
histogram(CT_r,edges)
legend('CT_r')
xlim([xl(25), xu(25)])

%% SVSA rear

subplot(2,13,22)
edges = linspace(xl(26), xu(26), nbins);
SVSA_r = zeros(1,length(pop));
for ii = 1:length(pop)
    SVSA_r(ii) = pop(ii).var(26);
end
histogram(SVSA_r,edges)
legend('SVSA_r')
xlim([xl(26), xu(26)])

%% Phi rear

subplot(2,13,23)
edges = linspace(xl(27), xu(27), nbins);
phi_r = zeros(1,length(pop));
for ii = 1:length(pop)
    phi_r(ii) = pop(ii).var(27);
end
histogram(phi_r,edges)
legend('phi_r')
xlim([xl(27), xu(27)])

%% xTROP rear

subplot(2,13,24)
edges = linspace(xl(28), xu(28), nbins);
xTRO_r = zeros(1,length(pop));
for ii = 1:length(pop)
    xTRO_r(ii) = pop(ii).var(28);
end
histogram(xTRO_r,edges)
legend('xTRO_r')
xlim([xl(28), xu(28)])

%% yTROP rear

subplot(2,13,25)
edges = linspace(xl(29), xu(29), nbins);
yTRO_r = zeros(1,length(pop));
for ii = 1:length(pop)
    yTRO_r(ii) = pop(ii).var(29);
end
histogram(yTRO_r,edges)
legend('yTRO_r')
xlim([xl(29), xu(29)])

%% zTROP rear

subplot(2,13,26)
edges = linspace(xl(30), xu(30), nbins);
zTRO_r = zeros(1,length(pop));
for ii = 1:length(pop)
    zTRO_r(ii) = pop(ii).var(30);
end
histogram(zTRO_r,edges)
legend('zTRO_r')
xlim([xl(30), xu(30)])
