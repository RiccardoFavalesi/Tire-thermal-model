%*************************************************************************
% Test Problem : 'Fit thermal model'
% Reference : [1] Deb K, Pratap A, Agarwal S, et al. A fast and elitist 
%   multiobjective genetic algorithm NSGA-II[J] . Evolutionary Computation. 
%   2002, 6(2): 182-197.
%*************************************************************************
%% intro
clear 
close all
clc
    
% k_shape = 0.615;
% k_shift = 0.02;
% T_ref = 80;
% k_pcpp = 0.1;
% k_refcpp = 1e5; %Pa;
% mu_base = 0.7;
% mu_peak = 2.25;

%% options initialization
currentFolder = pwd;
% GA_scripts = pwd + "\GA scripts";
% addpath(GA_scripts);
Model_Name = "Tread_carcass_Tire_thermal_model_simplified.slx";
Data_Name = "B2356raw9.mat";  % DA CAMBIARE ANCHE SU obj_fcn.m
options = nsgaopt();                    % create default options structure
data = load(Data_Name);
TSTC = data.TSTC;
indexi = 1;   %DA CAMBIARE ANCHE SU obj_fcn.m
indexf = length(TSTC);
firstindex = ceil(indexi/100);
stop_sim = floor(indexf/100);
%-----------------------------------------------------------------------------
options.OptName = 'Default'; % IMPORTANTE DEVE MATCHARE IL NOME DELLA CARTELLA
%-----------------------------------------------------------------------------


%% input
% options.popsize = 4;                    % population size   4
% options.maxGen  = 4;                    % max generation
% options.numVar  = 9;                    % number of design variables
% options.numObj  = 1;                    % number of objectives
% options.numCons = 0;                    % number of constraints
% options.vartype = ones(9,1);            % type

options.popsize = 10;                  % Larger population for stability
options.maxGen  = 10;                   % More generations for refinement
options.mutationRate = 0.5;            % Small mutation to avoid instability
options.crossoverRate = 0.9;           % High crossover for diversity
options.elitism = 2;                    % Preserve best solutions
options.numVar  = 9;                    % Number of design variables
options.numObj  = 1;                    % Number of objectives
options.numCons = 0;                    % Constraints (if needed)
options.vartype = ones(9,1);            % Variable types


%% lower bounds
options.lb = [  0.2,  ...  % var1 exponentialcoefficient_Fz
                0.1,  ...  % var2 exponentialcoefficient_p
                0.09, ...  % var3 a_coeff
                5000, ...  % var4 H_tread_road
                0.5, ...   % var5 H_ta_proportional
                0, ...     % var6 H_ta_constant
                50, ...  % var7 b_coeff
                500, ...   % var8 S_tread
                30];       % var 9 H_tc

%% upper bounds
options.ub = [  0.9,  ...  % var1
                0.9,  ...  % var2
                0.90, ...  % var3
                15000, ... % var4
                2.5, ...   % var5
                30, ...    % var6
                8000, ...  % var7
                2000, ...  % var8
                80];     

%% obj_fun
options.objfun = @obj_fcn;     % objective function handle

result = nsga2(options);       % begin the optimization!

%% ELABORAZIONE DATI:
for ii = 1:options.popsize
    if result.pops(end, ii).rank == 1
        ii_best = ii;
    end
end
best_individual = result.pops(end, ii_best).var;

exponentialcoefficient_Fz_best = best_individual(1);
exponentialcoefficient_p_best = best_individual(2);
a_coeff_best = best_individual(3);
H_tread_road_best = best_individual(4);
H_ta_proportional_best = best_individual(5);
H_ta_constant_best = best_individual(6);
b_coeff_best = best_individual(7);
S_tread_best = best_individual(8);
H_tc_best = best_individual(9);


% Model_Name = 'thermal_model_differential_simplified';
% % open_system(Model_Name);
% simIn = Simulink.SimulationInput(Model_Name);
% simIn(1:pop_size) = Simulink.SimulationInput(Model_Name);

%indexi = 1;
%indexf = length(TSTC);
% indexi = 200e2;
% indexf = 800e2;
timevals = data.ET(indexi:indexf);
dt = timevals(2) - timevals(1);
Re    = data.RE(indexi:indexf);   % [cm]
Re    = Re/100;              % [m]
omega = data.N(indexi:indexf);    % [rpm]
omega = omega*pi/30;         % [rad/s] 
Vx    = data.V(indexi:indexf);    % [kph] 
Vx    = Vx/3.6;              % [m/s]
Fy    = data.FY(indexi:indexf);   % [N]
Fx    = data.FX(indexi:indexf);   % [N]
Fz    = data.FZ(indexi:indexf);   % [N]
p_inf = data.P(indexi:indexf);    % [kPa]
p_inf = p_inf*1000;          % [Pa]
Vs = Vx - omega.*Re; %sliding speed (from Brush Model)
T_amb = data.AMBTMP(indexi:indexf);
T_road = data.RST(indexi:indexf);
force = sqrt(Fx.^2 + Fy.^2);
mu_d = force./Fz;
w_cp = 0.12; % [m]
exponentialcoefficient_Fz = exponentialcoefficient_Fz_best; % DA OTTIMIZZARE
exponentialcoefficient_p = exponentialcoefficient_p_best; % DA OTTIMIZZARE
a_coeff = a_coeff_best;   %it's 0.11 for fronts and 0.13 for rears % DA OTTIMIZZARE
H_tread_road      = H_tread_road_best; % [W/(m^2*K)] DA OTTIMIZZARE
H_ta_proportional = H_ta_proportional_best; % DA OTTIMIZZARE
H_ta_constant = H_ta_constant_best; % DA OTTIMIZZARE
b_coeff = b_coeff_best; % DA OTTIMIZZARE
H_tc = H_tc_best;
S_tread   = S_tread_best;           % [J/(kg*K)] DA OTTIMIZZARE
S_gas     = 1042;         % [J/(kg*K)]
M_tread   = 0.45;           % [kg]
scaling_Qambient = 1; %LEAVE 1, this parameter is only a gain to perform a sensitivity analysis on H_treadmabient, which is 10+2*vx
T_road = timeseries(T_road, timevals);
T_amb = timeseries(T_amb, timevals);
mu_d = abs(mu_d);
mu_d = timeseries(mu_d, timevals);
Fx = abs(Fx);
Fx = timeseries(Fx, timevals);
Fy = abs(Fy);
Fy = timeseries(Fy, timevals);
Fz = abs(Fz);
Fz = timeseries(Fz, timevals);
Vs = abs(Vs);
Vs = timeseries(Vs, timevals);
Vx = abs(Vx);
Vx = timeseries(Vx, timevals);
p_inf = timeseries(p_inf, timevals);
%NUOVI

%thermal_model_differential_simplified_new;
out = sim(Model_Name,'ReturnWorkspaceOutputs','on');

%%
figure();
plot(timevals,TSTC(indexi:indexf),'LineWidth',2);
hold on;
grid on;
plot(out.simout.Time,out.simout.Data,'LineWidth',2);
legend('data','model');
xlabel('time');
ylabel('temperature [°C]');

%%
aaa = length(timevals)
TEMP_modello = interp1(out.simout.Time, out.simout.Data, timevals,'linear', 'extrap');

errore = TSTC(1:aaa) - TEMP_modello;

figure
subplot(2,1,1)
plot(timevals, TEMP_modello, 'LineWidth',1.5);
hold on
plot(timevals, TSTC(1:aaa), 'LineWidth', 1.5)
rms_errore = rms(errore);
subplot(2,1,2)
plot(timevals, errore)
hold on
yline(rms_errore, 'LineWidth', 2)
linkaxes([subplot(2,1,1), subplot(2,1,2)], 'x');
results = best_individual;
results(1, length(results)+1) = rms_errore;
results(1, length(results)+1) = 30;


%%
MAPE = mape(TEMP_modello, TSTC)
mean_abs_err = mean(abs(errore))
rms_errore




