%% Intro
clear
close all
clc

nbins = 20;

tw = 1270;
wb = 1530;

obj_fcn_name = {'yaw rate overshoot', 'response time', 'ay max1', 'ay max2', 'USG', 'beta grad', 'phase delay', 'ay max3', 'max dist', 'ax min'};

%% Lower bounds

xl = [  25, ... %RCHf
        tw, ... %FVSAf
        10, ... %aKingPinf
       -20, ... %SRf
       350, ... %zUBJf
       280, ... %zLBJf
         6, ... %aCasterf
        10, ... %CTf
      wb/2, ... %SVSAf
       4.2, ... %fif
    wb-130, ... %xTROPf
       180, ... %zTROPf
        -8, ... %betaf
       -40, ... %theta_F_UCAf
        15, ... %theta_R_UCAf
       -40, ... %theta_F_LCAf
        15, ... %theta_R_LCAf
        50, ... %RCHr
        tw, ... %FVSAr
        10, ... %aKingPinr
       -20, ... %SRr
       350, ... %zUBJr
       280, ... %zLBJr
         2, ... %aCasterr
         5, ... %CTr
      wb/2, ... %SVSAr
       4.2, ... %fir
      -120, ... %xTROPr
  tw/2-100, ... %yTROPr
       220, ... %zTROPr
       -35, ... %theta_F_UCAr
        15, ... %theta_R_UCAr
       -35, ... %theta_F_LCAr
        15, ... %theta_R_LCAr
         1, ... %tube_front_A
         1, ... %tube_front_B
         1, ... %tube_front_C
         1, ... %tube_front_D
         1, ... %tube_rear_A
         1, ... %tube_rear_B
         1, ... %tube_rear_C
         1, ... %tube_rear_D
         1, ... %front_pullrod
         1, ... %rear_pushrod
         1, ... %front_tierod
         1, ... %rear_tierod
         0, ... %motorphi_F
         0];    %motorphi_R     % lower bound of x

%% Upper bounds

xu = [  75, ... %RCHf
      4*tw, ... %FVSAf
        17, ... %aKingPinf
        30, ... %SRf
       450, ... %zUBJf
       320, ... %zLBJf
        12, ... %aCasterf
        30, ... %CTf
      4*wb, ... %SVSAf
       8.5, ... %fif
     wb-70, ... %xTROPf
       230, ... %zTROPf
         8, ... %betaf
       -20, ... %theta_F_UCAf
        25, ... %theta_R_UCAf
       -20, ... %theta_F_LCAf
        25, ... %theta_R_LCAf
       125, ... %RCHr
    2.5*tw, ... %FVSAr
        17, ... %aKingPinr
        30, ... %SRr
       450, ... %zUBJr
       320, ... %zLBJr
         8, ... %aCasterr
        20, ... %CTr
      2*wb, ... %SVSAr
       8.5, ... %fir
       -80, ... %xTROPr
      tw/2, ... %yTROPr
       250, ... %zTROPr
       -15, ... %theta_F_UCAr
        25, ... %theta_R_UCAr
       -15, ... %theta_F_LCAr
        25, ... %theta_R_LCAr
         3, ... %tube_front_A
         3, ... %tube_front_B
         3, ... %tube_front_C
         3, ... %tube_front_D
         3, ... %tube_rear_A
         3, ... %tube_rear_B
         3, ... %tube_rear_C
         3, ... %tube_rear_D
         3, ... %front_pullrod
         3, ... %rear_pushrod
         3, ... %front_tierod
         3, ... %rear_tierod
        30, ... %motorphi_F
        30];    %motorphi_R]    % upper bound of x



%% Yaw Rate Overshoot

addpath('Results')
d = dir('Results');

mean_YRO = zeros(1,length(d)-2);
best_YRO = zeros(1,length(d)-2);
stdv_YRO = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    YRO = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        YRO(jj) = pop(jj).obj(1);
    end
    
    mean_YRO(ii-2) = mean(YRO);
    best_YRO(ii-2) = min(YRO);
    stdv_YRO(ii-2) = std(YRO);

end

subplot(2,5,1)
title('Yaw rate overshoot')
hold on
plot(mean_YRO, '-r', 'LineWidth', 2)
plot(best_YRO, '-b', 'LineWidth', 2)
yyaxis right
plot(stdv_YRO, '-g', 'LineWidth', 2)
grid on

%% Response time

addpath('Results')
d = dir('Results');

mean_RT = zeros(1,length(d)-2);
best_RT = zeros(1,length(d)-2);
stdv_RT = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    RT = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        RT(jj) = pop(jj).obj(2);
    end
    
    mean_RT(ii-2) = mean(RT);
    best_RT(ii-2) = min(RT);
    stdv_RT(ii-2) = std(RT);

end

subplot(2,5,2)
title('Response time')
hold on
plot(mean_RT, '-r', 'LineWidth', 2)
plot(best_RT, '-b', 'LineWidth', 2)
yyaxis right
plot(stdv_RT, '-g', 'LineWidth', 2)
grid on

%% Ay max 1

addpath('Results')
d = dir('Results');

mean_ay1 = zeros(1,length(d)-2);
best_ay1 = zeros(1,length(d)-2);
stdv_ay1 = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    ay1 = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        ay1(jj) = pop(jj).obj(3);
    end
    
    mean_ay1(ii-2) = mean(ay1);
    best_ay1(ii-2) = min(ay1);
    stdv_ay1(ii-2) = std(ay1);

end

subplot(2,5,3)
title('Ay min 1')
hold on
plot(mean_ay1, '-r', 'LineWidth', 2)
plot(best_ay1, '-b', 'LineWidth', 2)
yyaxis right
plot(stdv_ay1, '-g', 'LineWidth', 2)
grid on

%% Ay max 2

addpath('Results')
d = dir('Results');

mean_ay2 = zeros(1,length(d)-2);
best_ay2 = zeros(1,length(d)-2);
stdv_ay2 = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    ay2 = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        ay2(jj) = pop(jj).obj(4);
    end
    
    mean_ay2(ii-2) = mean(ay2);
    best_ay2(ii-2) = min(ay2);
    stdv_ay2(ii-2) = std(ay2);

end

subplot(2,5,4)
title('Ay min 2')
hold on
plot(mean_ay2, '-r', 'LineWidth', 2)
plot(best_ay2, '-b', 'LineWidth', 2)
ylim([-22 -21.8])
yyaxis right
plot(stdv_ay2, '-g', 'LineWidth', 2)
grid on

%% USG

addpath('Results')
d = dir('Results');

mean_USG = zeros(1,length(d)-2);
best_USG = zeros(1,length(d)-2);
stdv_USG = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    USG = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        USG(jj) = pop(jj).obj(5);
    end
    
    mean_USG(ii-2) = mean(USG);
    best_USG(ii-2) = min(USG);
    stdv_USG(ii-2) = std(USG);

end

subplot(2,5,5)
title('USG')
hold on
plot(mean_USG, '-r', 'LineWidth', 2)
plot(best_USG, '-b', 'LineWidth', 2)
ylim([4.21*1e-3 4.25*1e-3])
yyaxis right
plot(stdv_USG, '-g', 'LineWidth', 2)
grid on

%% SSG

addpath('Results')
d = dir('Results');

mean_SSG = zeros(1,length(d)-2);
best_SSG = zeros(1,length(d)-2);
stdv_SSG = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    SSG = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        SSG(jj) = pop(jj).obj(6);
    end
    
    mean_SSG(ii-2) = mean(SSG);
    best_SSG(ii-2) = min(SSG);
    stdv_SSG(ii-2) = std(SSG);

end

subplot(2,5,6)
title('SSG')
hold on
plot(mean_SSG, '-r', 'LineWidth', 2)
plot(best_SSG, '-b', 'LineWidth', 2)
ylim([0.5*1e-3 1.5*1e-3])
yyaxis right
plot(stdv_SSG, '-g', 'LineWidth', 2)
grid on

%% PHD

addpath('Results')
d = dir('Results');

mean_PHD = zeros(1,length(d)-2);
best_PHD = zeros(1,length(d)-2);
stdv_PHD = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    PHD = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        PHD(jj) = pop(jj).obj(7);
    end
    
    mean_PHD(ii-2) = mean(PHD);
    best_PHD(ii-2) = min(PHD);
    stdv_PHD(ii-2) = std(PHD);

end

subplot(2,5,7)
title('PHD')
hold on
plot(mean_PHD, '-r', 'LineWidth', 2)
plot(best_PHD, '-b', 'LineWidth', 2)
yyaxis right
plot(stdv_PHD, '-g', 'LineWidth', 2)
grid on

%% Ay max 3

addpath('Results')
d = dir('Results');

mean_ay3 = zeros(1,length(d)-2);
best_ay3 = zeros(1,length(d)-2);
stdv_ay3 = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    ay3 = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        ay3(jj) = pop(jj).obj(8);
    end
    
    mean_ay3(ii-2) = mean(ay3);
    best_ay3(ii-2) = min(ay3);
    stdv_ay3(ii-2) = std(ay3);

end

subplot(2,5,8)
title('ay3')
hold on
plot(mean_ay3, '-r', 'LineWidth', 2)
plot(best_ay3, '-b', 'LineWidth', 2)
yyaxis right
plot(stdv_ay3, '-g', 'LineWidth', 2)
grid on

%% Max dist

addpath('Results')
d = dir('Results');

mean_md = zeros(1,length(d)-2);
best_md = zeros(1,length(d)-2);
stdv_md = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    md = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        md(jj) = pop(jj).obj(9);
    end
    
    mean_md(ii-2) = mean(md);
    best_md(ii-2) = min(md);
    stdv_md(ii-2) = std(md);

end

subplot(2,5,9)
title('max distance')
hold on
plot(mean_md, '-r', 'LineWidth', 2)
plot(best_md, '-b', 'LineWidth', 2)
yyaxis right
plot(stdv_md, '-g', 'LineWidth', 2)
grid on

%% Ax min

addpath('Results')
d = dir('Results');

mean_ax = zeros(1,length(d)-2);
best_ax = zeros(1,length(d)-2);
stdv_ax = zeros(1,length(d)-2);

for ii = 3:length(d)

    load(d(ii).name);

    ax = zeros(1,length(pop));
    
    for jj = 1:length(pop)
        ax(jj) = pop(jj).obj(10);
    end
    
    mean_ax(ii-2) = mean(ax);
    best_ax(ii-2) = min(ax);
    stdv_ax(ii-2) = std(ax);

end

subplot(2,5,10)
title('ax min')
hold on
plot(mean_ax, '-r', 'LineWidth', 2)
plot(best_ax, '-b', 'LineWidth', 2)
yyaxis right
plot(stdv_ax, '-g', 'LineWidth', 2)
grid on