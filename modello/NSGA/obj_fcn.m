function [fit, err] = obj_fcn(var,pop_size,M)
    %% guida
    % fit: vettore riga contenente le objective functions
    
    % esempio a caso:
    % f1 = x(2)^2+2*x(3) 
    % f2 = x(1)/8 - 4*x(4)
    % fit = [f1 f2]
    
    % err: vettore riga contenente la violazione dei vincoli c
    % c: vettore riga contenente i vari vincoli espressi come uguaglianze di
    % modo che c(i) risulti >0 solo se il vincolo non viene rispettato
    
    % esempio a caso: se voglio -x(2) > -(9*x(1))+6 e x(2) < 9*x(1)+1
    %  c(1,1) = -x(2)-(9*x(1))+6;
    %  c(1,2) = x(2)-9*x(1)+1;
    %  err = (c>0).*c;
    
    % se non ci sono vincoli:
    % err = zeros(1,1);
    
    %% assegnazione variabili e alloco

    of = zeros(pop_size, M);        % allocazione memoria
%     data                            % inizializzazione variabili multibody

    Model_Name = "Tread_carcass_Tire_thermal_model_simplified.slx";
    Data_Name = "B2356raw9.mat";
    open_system(Model_Name)
%     set_param(Model_Name,'InitFcn','');        %as a workaround it is cleared and restored afterwars

    %% Prove in parallelo

%     maneuvers   = ["step" "ramp" "sine" "acc_brk"];
%     n_driving_mode = numel(maneuvers);

%     vars_repeat = repmat(var', n_driving_mode,1);    % ripete l'array di variabili per ogni prova da fare    
%     driving_mode_repeat = repelem(maneuvers, pop_size);   % ripete l'array che indica la manovra da eseguire

    simIn(1:pop_size) = Simulink.SimulationInput(Model_Name);  % pre-allocates simIn memory

    for ii = 1:pop_size
        data = load(Data_Name);
        TSTC = data.TSTC;
        indexi = 1;
        indexf = length(TSTC);
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
        simIn(ii) = simIn(ii).setVariable('w_cp', w_cp);
        exponentialcoefficient_Fz = var(ii).exponentialcoefficient_Fz; % DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('exponentialcoefficient_Fz', exponentialcoefficient_Fz);
        exponentialcoefficient_p = var(ii).exponentialcoefficient_p; % DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('exponentialcoefficient_p', exponentialcoefficient_p);
        a_coeff = var(ii).a_coeff;   %it's 0.11 for fronts and 0.13 for rears % DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('a_coeff', a_coeff);
        H_tread_road      = var(ii).H_tread_road; % [W/(m^2*K)] DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('H_tread_road', H_tread_road);
        H_ta_proportional = var(ii).H_ta_proportional; % DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('H_ta_proportional', H_ta_proportional);
        H_ta_constant = var(ii).H_ta_constant; % DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('H_ta_constant', H_ta_constant);
        b_coeff = var(ii).b_coeff; % DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('b_coeff', b_coeff);
        S_tread   = var(ii).S_tread;           % [J/(kg*K)] DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('S_tread', S_tread);
        S_gas     = 1042;         % [J/(kg*K)]
        simIn(ii) = simIn(ii).setVariable('S_gas', S_gas);
        H_tc = var(ii).H_tc; % DA OTTIMIZZARE
        simIn(ii) = simIn(ii).setVariable('H_tc', H_tc);
        M_tread   = 0.35;           % [kg]
        simIn(ii) = simIn(ii).setVariable('M_tread', M_tread);
        scaling_Qambient = 1; %LEAVE 1, this parameter is only a gain to perform a sensitivity analysis on H_treadmabient, which is 10+2*vx
        simIn(ii) = simIn(ii).setVariable('scaling_Qambient', scaling_Qambient);

        T_road = timeseries(T_road, timevals);
        simIn(ii) = simIn(ii).setVariable('T_road', T_road);
        T_amb = timeseries(T_amb, timevals);
        simIn(ii) = simIn(ii).setVariable('T_amb', T_amb);
        mu_d = abs(mu_d);
        mu_d = timeseries(mu_d, timevals);
        simIn(ii) = simIn(ii).setVariable('mu_d', mu_d);
        Fx = abs(Fx);
        Fx = timeseries(Fx, timevals);
        Fy = abs(Fy);
        Fy = timeseries(Fy, timevals);
        Fz = abs(Fz);
        Fz = timeseries(Fz, timevals);
        simIn(ii) = simIn(ii).setVariable('Fz', Fz);
        Vs = abs(Vs);
        Vs = timeseries(Vs, timevals);
        simIn(ii) = simIn(ii).setVariable('Vs', Vs);
        Vx = abs(Vx);
        Vx = timeseries(Vx, timevals);
        simIn(ii) = simIn(ii).setVariable('Vx', Vx);
        p_inf = timeseries(p_inf, timevals);
        simIn(ii) = simIn(ii).setVariable('p_inf', p_inf);
%         init_OpenLoop;

%         simIn(ii)=simIn(ii).setVariable('MB' , MB);   % imports the modified Smi in the simulation ???
%         simIn(ii)=simIn(ii).setVariable('TV' , TV);   % imports the modified Smi in the simulation ???
        
    end
    clear ii

    %% run simulation
    save_system(Model_Name)
    out = sim(simIn);     % simulation
%     set_param(Model_Name,'InitFcn','init_OpenLoop'); % Restore original settings

    %% Step steer
    for ii = 1:pop_size

%         if ~isempty(out(ii).ErrorMessage) || any(out(ii).Timeout.Data)
%                 of(ii,1) = 10;
%                 of(ii,2) = 10;
%                 of(ii,3) = 0;
%                 continue
%         end

        
        % load signals
%         steering_angle = out(ii).MB.driver.SW.F.Data;                       % [deg]
%         yaw_rate       = (pi/180)*out(ii).MB.Dir.Chassis.COG.yaw_rate.Data; % [rad/s]
%         Ay             = out(ii).MB.Dir.Chassis.COG.ay.Data;                % [m/s^2]
%         time           = out(ii).tout;                                      % [s]
        time_model = out(ii).simout.Time;
        T_model = out(ii).simout.Data; % [°C]

        RSS = 0;
        for jj = 1:(length(T_model)-1)
            RSS = RSS + (TSTC(floor(time_model(jj)/dt)+1) - T_model(jj))^2;
        end
        RSS = RSS / jj; % perché uso variable step in Simulink
        clear jj;

%         of(ii,3) = max(abs(Ay));        % peak lateral acceleration
        of(ii,1) = RSS;

        % plot
%         figure
%         out(ii).logsout{1}.Values.vehBUS.cntrlBUS.steering.plot
%         figure
%         out(ii).logsout{1}.Values.vehBUS.COG_MOTION.sensing.yaw_velocity.plot
%         figure
%         out(ii).logsout{1}.Values.vehBUS.COG_MOTION.sensing.ay.plot
    end
    
    %% assegnazione of e constraints
%     fit = [of(:,1), of(:,2), -of(:,3), -of(:,4), of(:,5), of(:,6), of(:,7), -of(:,8), -of(:,9), -of(:,10)];
    fit = [of(:, 1)];

%     c(:,1) = - of(:,3) + 2.00*9.81;
    c = zeros(pop_size, 0);

    err = (c>0).*c;
end