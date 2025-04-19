function [pop, state] = evaluate(opt, pop, state, varargin)
% Function: [pop, state] = evaluate(opt, pop, state, varargin)
% Description: Evaluate the objective functions of each individual in the
%   population.
%
%         LSSSSWC, NWPU
%    Revision: 1.0  Data: 2011-04-20
%*************************************************************************

N = length(pop);
allTime = zeros(N, 1);  % allTime : use to calculate average evaluation times

%*************************************************************************
% Evaluate objective function in parallel
%*************************************************************************
if( strcmpi(opt.useParallel, 'yes') == 1 )
    curPoolsize = parpool('size');

    % There isn't opened worker process
    if(curPoolsize == 0)
        if(opt.poolsize == 0)
            parpool open local
        else
            parpool(opt.poolsize)
        end
    % Close and recreate worker process
    else
        if(opt.poolsize ~= curPoolsize)
            parpool close
            parpool(opt.poolsize)
        end
    end

    parfor i = 1:N
        fprintf('\nEvaluating the objective function... Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
        [pop(i), allTime(i)] = evalIndividual(pop(i), opt.objfun, varargin{:});
    end

%*************************************************************************
% Evaluate objective function in serial
%*************************************************************************
else
    fprintf('\nEvaluating the objective function... Generation: %d / %d \n', state.currentGen, opt.maxGen);
    [pop, allTime] = evalIndividual(pop, opt.objfun, varargin{:});
end

%*************************************************************************
% Statistics
%*************************************************************************
state.avgEvalTime   = sum(allTime) / length(allTime);
state.evaluateCount = state.evaluateCount + length(pop);

function [indi, evalTime] = evalIndividual(indi, objfun, varargin)
% Function: [indi, evalTime] = evalIndividual(indi, objfun, varargin)
% Description: Evaluate one objective function.
%
%         LSSSSWC, NWPU
%    Revision: 1.1  Data: 2011-07-25
%*************************************************************************

tStart = tic;
V = 15;
M = 2; 
[~, pop_size] = size(indi);
for ii = 1:pop_size
%     x(ii).perf1     = indi(ii).var(1);
%     x(ii).perf2     = indi(ii).var(2);
%     x(ii).perf3     = indi(ii).var(3);
%     x(ii).perf4     = indi(ii).var(4);
%     x(ii).perf5     = indi(ii).var(5);
%     x(ii).perf6     = indi(ii).var(6);
%     x(ii).f0_t      = indi(ii).var(7);
%     x(ii).h_t       = indi(ii).var(8);
%     x(ii).f0_b      = indi(ii).var(9);
%     x(ii).h_b       = indi(ii).var(10);
%     x(ii).Lat_T_risp    = indi(ii).var(11);
%     x(ii).Lon_T_risp    = indi(ii).var(12);
%     x(ii).T_inte    = indi(ii).var(13);
%     x(ii).c_AD      = indi(ii).var(14);
%     x(ii).c_AT      = indi(ii).var(15);
    x(ii).p1 = indi(ii).var(1);
    x(ii).p2 = indi(ii).var(2);
    x(ii).p3 = indi(ii).var(3);
    x(ii).S_tread = indi(ii).var(4);
    x(ii).h_tt = indi(ii).var(5);
    x(ii).cs = indi(ii).var(6);
    x(ii).a_cp = indi(ii).var(7);
    %x(ii).S_tread = indi(ii).var(8);
    %x(ii).H_tc = indi(ii).var(9);
end

%% run
[y, cons] = objfun(x, pop_size, M);
evalTime = toc(tStart);

%% Save the objective values and constraint violations

for ii = 1:pop_size
    indi(ii).obj = y(ii,:);
    indi(ii).cons = cons(ii,:);
    if( ~isempty(indi(ii).cons) )
        idx = find( cons(ii,:) );
        if( ~isempty(idx) )
            indi(ii).nViol = length(idx);
            indi(ii).violSum = sum( abs(cons(ii,:)) );
        else
            indi(ii).nViol = 0;
            indi(ii).violSum = 0;
        end
    end
end


