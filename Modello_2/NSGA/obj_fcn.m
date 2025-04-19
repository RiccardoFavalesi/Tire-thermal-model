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

    Model_Name = "New_tread_carcass_thermal_model.slx";
    Data_Name = "B2356raw8.mat";
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
        indexi = 1;   %DA CAMBIARE ANCHE SU obj_fcn.m
        indexf = length(TSTC);
        timevals = data.ET(indexi:indexf);
        dt = timevals(2) - timevals(1);
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
        simIn(ii) = simIn(ii).setVariable('w_cp', w_cp);

        p1 = var(ii).p1;
        simIn(ii) = simIn(ii).setVariable('p1', p1);
        p2 = var(ii).p2;
        simIn(ii) = simIn(ii).setVariable('p2', p2);
        p3 = var(ii).p3;
        simIn(ii) = simIn(ii).setVariable('p3', p3);
        S_tread = var(ii).S_tread;
        simIn(ii) = simIn(ii).setVariable('S_tread', S_tread);
        h_tt = var(ii).h_tt;
        simIn(ii) = simIn(ii).setVariable('h_tt', h_tt);
        cs = var(ii).cs;
        simIn(ii) = simIn(ii).setVariable('cs', cs);
        a_cp = var(ii).a_cp;
        simIn(ii) = simIn(ii).setVariable('a_cp', a_cp);
        M_tread   = 0.30;           % [kg]
        simIn(ii) = simIn(ii).setVariable('M_tread', M_tread);
        SA = timeseries(SA, timevals);
        simIn(ii) = simIn(ii).setVariable('SA', SA);
        IA = timeseries(IA, timevals);%camber angle
        simIn(ii) = simIn(ii).setVariable('IA', IA);
        SL = timeseries(SA, timevals);
        simIn(ii) = simIn(ii).setVariable('SL',SL);
        T_amb = timeseries(T_amb, timevals);
        simIn(ii) = simIn(ii).setVariable('T_amb', T_amb);
        mu_d = timeseries(mu_d, timevals);
        simIn(ii) = simIn(ii).setVariable('mu_d', mu_d);
        Fx = timeseries(Fx, timevals);
        simIn(ii) = simIn(ii).setVariable('Fx', Fx);
        Fy = timeseries(Fy, timevals);
        simIn(ii) = simIn(ii).setVariable('Fy', Fy);
        Fz = timeseries(Fz, timevals);
        simIn(ii) = simIn(ii).setVariable('Fz', Fz);
        Vs = timeseries(Vs, timevals);
        simIn(ii) = simIn(ii).setVariable('Vs', Vs);
        Vx = timeseries(Vx, timevals);
        simIn(ii) = simIn(ii).setVariable('Vx', Vx);
        p_inf = timeseries(p_inf, timevals);
        simIn(ii) = simIn(ii).setVariable('p_inf', p_inf);
        Rl = timeseries(Rl, timevals);
        simIn(ii) = simIn(ii).setVariable('Rl', Rl);
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