%*************************************************************************
% Test Problem : 'Fit thermal model'
% Reference : [1] Deb K, Pratap A, Agarwal S, et al. A fast and elitist 
%   multiobjective genetic algorithm NSGA-II[J] . Evolutionary Computation. 
%   2002, 6(2): 182-197.
%*************************************************************************
%% intro

clc
clear all
close all
%% options initialization
currentFolder = pwd;
% GA_scripts = pwd + "\GA scripts";
% addpath(GA_scripts);  
Model_Name = "New_tread_carcass_thermal_model.slx";
Data_Name = "B2356raw8.mat";  % DA CAMBIARE ANCHE SU obj_fcn.m
options = nsgaopt();                    % create default options structure
data = load(Data_Name);

TSTC = data.TSTC;
indexi = 1;   %DA CAMBIARE ANCHE SU obj_fcn.m
indexf = length(TSTC);
timevals = data.ET(indexi:indexf);
dt = timevals(2) - timevals(1);
firstindex = ceil(indexi/100);
stop_sim = floor(indexf/100);
%-----------------------------------------------------------------------------
options.OptName = 'Default'; % IMPORTANTE DEVE MATCHARE IL NOME DELLA CARTELLA
%-----------------------------------------------------------------------------


SA = data.SA(indexi:indexf); %slip angle
IA = data.IA(indexi:indexf); %camber angle
SL = data.SL(indexi:indexf); %slip ratio
Re    = data.RE(indexi:indexf);   % [cm]
Re    = Re/100;              % [m]
Rl    = data.RL(indexi:indexf);   % [cm]
Rl    = Rl/100;              % [m]  loaded radius
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
T_track = data.RST(indexi:indexf);
force = sqrt(Fx.^2 + Fy.^2);
mu_d = force./Fz;
w_cp = 0.12; % [m]


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
options.numVar  = 7;                    % Number of design variables
options.numObj  = 1;                    % Number of objectives
options.numCons = 0;                    % Constraints (if needed)
options.vartype = ones(7,1);            % Variable types

%% lower bounds
options.lb = [  0.2,  ...  % var1 exponentialcoefficient_Fz
                0.1,  ...  % var2 exponentialcoefficient_p
                0.09, ...  % var3 a_coeff
                5000, ...  % var4 H_tread_road
                0.5, ...   % var5 H_ta_proportional
                0, ...     % var6 H_ta_constant
                50];  % var7 b_coeff

  

%% upper bounds
options.ub = [  0.9,  ...  % var1
                0.9,  ...  % var2
                0.90, ...  % var3
                15000, ... % var4
                2.5, ...   % var5
                30, ...    % var6
                8000];      % var7
                    

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

p1_best = best_individual(1);
p2_best = best_individual(2);
p3_best = best_individual(3);
S_tread_best = best_individual(4);
h_tt_best = best_individual(5);
cs_best = best_individual(6);
a_cp_best = best_individual(7);
%S_tread_best = best_individual(8);
%H_tc_best = best_individual(9);


p1 = p1_best; % DA OTTIMIZZARE
p2= p2_best; % DA OTTIMIZZARE
p3 = p3_best;   %it's 0.11 for fronts and 0.13 for rears % DA OTTIMIZZARE
S_tread      = S_tread_best; % [W/(m^2*K)] DA OTTIMIZZARE
h_tt = h_tt_best; t% DA OTTIMIZZARE
cs = cs_best; % DA OTTIMIZZARE
a_cp = a_cp_best; % DA OTTIMIZZARE
%H_tc = H_tc_best;
%S_tread   = S_tread_best;           % [J/(kg*K)] DA OTTIMIZZARE
S_gas     = 1042;         % [J/(kg*K)]
M_tread   = 0.30;           % [kg]


SA = abs(SA);
SA = timeseries(SA, timevals);
IA = abs(IA);
IA = timeseries(IA, timevals);%camber angle
SL = abs(SL);
SL = timeseries(SL, timevals);
T_track = timeseries(T_track, timevals);
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
Rl = abs(Rl);
Rl = timeseries(Rl, timevals);


%thermal_model_differential_simplified_new;
out = sim(Model_Name,'ReturnWorkspaceOutputs','on');