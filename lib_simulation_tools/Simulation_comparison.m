function [x, xd, t, xT, x_obs] = Simulation_comparison(x0,xT,fn_handle,varargin)
%
% This function simulates motion that were learnt using SEDS, which defines
% a motins as a nonlinear time-independent asymptotically stable dynamical
% systems:
%                               xd=f(x)
%
% where x is an arbitrary d dimensional variable, and xd is its first time
% derivative.
%
% The function can be called using:
%       [x xd t]=Simulation(x0,xT,Priors,Mu,Sigma)
%
% or
%       [x xd t]=Simulation(x0,xT,Priors,Mu,Sigma,options)
%
% to also send a structure of desired options.
%
% Inputs -----------------------------------------------------------------
%   o x:       d x N matrix vector representing N different starting point(s)
%   o xT:      d x 1 Column vector representing the target point
%   o fn_handle:  A handle function that only gets as input a d x N matrix,
%                 and returns the output matrix of the same dimension. Note
%                 that the output variable is the first time derivative of
%                 the input variable.
%
%   o options: A structure to set the optional parameters of the simulator.
%              The following parameters can be set in the options:
%       - .dt:      integration time step [default: 0.02]
%       - .i_max:   maximum number of iteration for simulator [default: i_max=1000]
%       - .plot     setting simulation graphic on (true) or off (false) [default: true]
%       - .tol:     A positive scalar defining the threshold to stop the
%                   simulator. If the motions velocity becomes less than
%                   tol, then simulation stops [default: 0.001]
%       - .perturbation: a structure to apply pertorbations to the robot.
%                        This variable has the following subvariables:
%       - .perturbation.tf:   A positive scalar defining the final time for
%                             the perturbations. This variable is necessary
%                             only when the type is set to 'tcp' or 'rcp'.
%       - .perturbation.dx:   A d x 1 vector defining the perturbation's
%                             magnitude. In 'tdp' and 'rdp', it simply
%                             means a relative displacement of the
%                             target/robot with the vector dx. In 'tcp' and
%                             'rcp', the target/robot starts moving with
%                             the velocity dx.
%       - .w:                 A scalar defining the angular velocity
%
% Outputs ----------------------------------------------------------------
%   o x:       d x T x N matrix containing the position of N, d dimensional
%              trajectories with the length T.
%
%   o xd:      d x T x N matrix containing the velocity of N, d dimensional
%              trajectories with the length T.
%
%   o t:       1 x N vector containing the trajectories' time.
%
%   o xT:      A matrix recording the change in the target position. Useful
%              only when 'tcp' or 'tdp' perturbations applied.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2010 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The program is free for non-commercial academic use. Please contact the
% author if you are interested in using the software for commercial purposes.
% The software must not be modified or distributed without prior permission
% of the authors. Please acknowledge the authors in any academic publications
% that have made use of this code or part of it. Please use this BibTex
% reference:
% 

%% parsing inputs
if isempty(varargin)
    options = check_options();
else
    options = check_options(varargin{1}); % Checking the given options, and add ones are not defined.
end

if ~isfield(options,'obstacleAvoidanceFunction') % Obstacle avoidance function
    fprintf('Default function handle \n')
    obsFunc_handle = @(x,xd,obs,varargin) obs_modulation_convergence(x,xd,obs, varargin); % INDEX 2
else
    obsFunc_handle = options.obstacleAvoidanceFunction;
end

if options.model == 2 %2nd order
    d=size(x0,1)/2; %dimension of the model
    if isempty(xT)
        xT = zeros(2*d,1);
    elseif 2*d~=size(xT,1)
        disp('Error: length(x0) should be equal to length(xT)!')
        x=[];xd=[];t=[];
        return
    end
else
    d=size(x0,1); %dimension of the model
    if isempty(xT)
        xT = zeros(d,1);
    elseif d~=size(xT,1)
        disp('Error: length(x0) should be equal to length(xT)!')
        x=[];xd=[];t=[];
        return
    end
end

if ~isfield(options,'saveAnimation')
    options.saveAnimation = false; % Default value
end

if options.saveAnimation
    figName = get(gcf,'Name');
    
    % Initialization of avi video object
    vidObj = VideoWriter(strcat('animations/', figName, '.avi')); % TODO
    
    %-- mp4 not working..
%     vidObj = VideoWriter(strcat('animations/', figName, '.mp4'), 'FileFormat','mp4');
    vidObj.Quality = 100;
    vidObj.FrameRate = 20;
    open(vidObj);
end
%% setting initial values
nbSPoint=size(x0,2); %number of starting points. This enables to simulatneously run several trajectories
N_pl=2; % Number of plots/simulation

convergenceReached = false;

if isfield(options,'obstacle') && ~isempty(options.obstacle) %there is obstacle
    obs_bool = true;
    obs = options.obstacle;
    for n=1:length(obs)
        x_obs{n} = obs{n}.x0;
        if ~isfield(obs{n},'x_center')
            obs{n}.x_center = zeros(d,1);
        end
        
        if ~isfield(obs{n},'extra')
            obs{n}.extra.ind = 2;
            obs{n}.extra.C_Amp = 0.01;
            obs{n}.extra.R_Amp = 0.0;
        else
            if ~isfield(obs{n}.extra,'ind')
                obs{n}.extra.ind = 2;
            end
            if ~isfield(obs{n}.extra,'C_Amp')
                obs{n}.extra.C_Amp = 0.01;
            end
            if ~isfield(obs{n}.extra,'R_Amp')
                obs{n}.extra.R_Amp = 0.0;
            end
        end
    end
    b_contour = zeros(1,nbSPoint);
else
    obs_bool = false;
    obs = [];
    x_obs = NaN;
end

%initialization
x = [];
for i=1:nbSPoint
    for ii=1:N_pl
        x{ii}(:,1,i) = x0(:,i);
    end
end
xd = [];
if options.model == 2 %2nd order
    for ff=1:N_pl
        xd{ff} = zeros(d,1,nbSPoint);
    end
else
    for ff=1:N_pl
        xd{ff} = zeros(size(x{ff}));
    end
end
if size(xT) == size(x0)
    XT = xT;
else
    XT = repmat(xT,1,nbSPoint); %a matrix of target location (just to simplify computation)
end
            
t=0; %starting time

if options.plot %plotting options
    if isfield(options,'figure')
        %sp.fig = options.figure;
        sp = [];
        sp{1}.fig = subplot(1,2,1);
        title('Local Modulation Matrix')
        
        sp{2}.fig = subplot(1,2,2);
        title('Inspired by Fluid Dynamics')
    else
        sp = [];
    end
    
    if d>1
        for ff=1:N_pl
            sp{ff} = subplot_results('i',sp{ff},x{ff},xT,obs);
        end
    else
        warning('No double plot implemented \n')
        for ff=1:N_pl
            sp{ff} = subplot_results('i',sp{ff},[x{ff};0],[xT;0],obs);
        end
    end
end

if isfield(options, 'attractor') % if no attractor -> 'None'
    attractor = options.attractor;
else
    attractor = [0;0];
end

if isfield(options, 'DS_type') % if no attractor -> 'None'
    ds_type = options.ds_type;
else
    ds_type = 'linear';
end

%% Simulation
iSim=1;
while true
    %Finding xd using fn_handle.
    for ff=1:N_pl
        if options.timeDependent
            tt = repmat(t(iSim),1,nbSPoint);
            if nargin(fn_handle) == 1
                xd{ff}(:,iSim,:)=reshape(fn_handle([squeeze(x{ff}(:,iSim,:))-XT;tt]),[d 1 nbSPoint]);
            else
                xd{ff}(:,iSim,:)=reshape(fn_handle(tt,squeeze(x{ff}(:,iSim,:))-XT),[d 1 nbSPoint]);
            end
        else
            xd{ff}(:,iSim,:)=reshape(fn_handle(squeeze(x{ff}(:,iSim,:))-XT),[d 1 nbSPoint]);
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This part if for the obstacle avoidance module
    if obs_bool
        % Draw obstacles
        [~, x_obs_sf] = obs_draw_ellipsoid(obs,50);
        
        w_obs = zeros( 1, length(obs) );
        xd_obs = zeros( d, length(obs) );

        timeVariant_simu = false;
        % applying perturbation on the obstacles
        for n=1:length(obs)
            % Initialize Obstacles
            w_obs(n) = 0;
            xd_obs(:,n) = [0;0];
            if isfield(obs{n},'perturbation')
                timeVariant_simu = true;
                if iSim >= round(obs{n}.perturbation.t0/options.dt)+1 && iSim <= round(obs{n}.perturbation.tf/options.dt) && length(obs{n}.perturbation.dx)==d
                    x_obs{n}(:,end+1) = x_obs{n}(:,end) + obs{n}.perturbation.dx*options.dt;
                    obs{n}.x0 = x_obs{n}(:,end);
                    xd_obs(:,n) = obs{n}.perturbation.dx;
                    
                    if isfield(obs{n}.perturbation,'w') % Check rotational rate
                        w_obs(n) = obs{n}.perturbation.w;
                    end
                else
                    x_obs{n}(:,end+1) = x_obs{n}(:,end);
                end
            else
            end
        end
        
        if and(or(timeVariant_simu, iSim == 1), length(obs)>1)  % Caclulate dynamic center
            % Check wheter there is an overlapping of obstacles
            [obs, intersection_obs]  = obs_common_section(obs, x_obs_sf);
            obs = calculate_dynamic_center(obs, x_obs_sf, intersection_obs);
            % TODO -- For long simulation and far away obsacle, dynamic
            % center suddendly stops to update. Find bug. Probs in
            % Simulation file
            if options.plot
                for n = 1:length(obs)
                    subplot_results('center_stat',sp{1},x{2},xT,n, obs); % Plot new center                    
                    subplot_results('center_dyn',sp{2},x{2},xT,n, obs); % Plot new center
                end
            end
        elseif length(obs) == 1
            obs{1}.x_center_dyn = obs{1}.x0;
            subplot_results('center_stat',sp{1},x{1},xT,n, obs); % Plot new center -- only for ff=2
            subplot_results('center_dyn',sp{2},x{2},xT,n, obs); % Plot new center -- only for ff=2
        end
        
        % TODO -- Redrawing still seems to have a delay of obstacle center,
        % how change this!?
        for n = 1:length(obs)
            if isfield(obs{n},'perturbation') % Only redraw moving obstacle
                 if iSim >= round(obs{n}.perturbation.t0/options.dt)+1 && iSim <= round(obs{n}.perturbation.tf/options.dt) && length(obs{n}.perturbation.dx)==d
                     for ff=1:N_pl
                        subplot_results('o',sp{ff},x{ff},xT,n,obs{n}.perturbation.dx*options.dt, obs);
                     end
                end
            end
        end
        
        for j=1:nbSPoint
            [xd{1}(:,iSim,j), b_contour(j), ~, compTime_temp] = obs_modulation_ellipsoid(x{1}(:,iSim,j),xd{1}(:,iSim,j),obs,b_contour(j),xd_obs, w_obs);
            [xd{2}(:,iSim,j), ~, compTime_temp] = obsFunc_handle(x{2}(:,iSim,j),xd{2}(:,iSim,j),obs, xd_obs, w_obs, attractor, ds_type);
        end
        
        for n=1:length(obs) % integration of object (linear motion!) -- first order.. 
            obs{n}.th_r(:) =  obs{n}.th_r + w_obs(n)*options.dt;
            %x_obs{n}(:,end+1) = x_obs{n}(:,end);
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Integration
    
    for ff=1:N_pl
        if options.model == 2 %2nd order
            x{ff}(d+1:2*d,iSim+1,:)=x{ff}(d+1:2*d,iSim,:)+xd{ff}(:,iSim,:)*options.dt;
            x{ff}(1:d,iSim+1,:)=x{ff}(1:d,iSim,:)+x{ff}(d+1:2*d,iSim,:)*options.dt;
        else
            x{ff}(:,iSim+1,:)=x{ff}(:,iSim,:)+xd{ff}(:,iSim,:)*options.dt;
        end
    end
    t(iSim+1)=t(iSim)+options.dt;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Applying perturbation if any
    switch options.perturbation.type
        case 'tdp' %applying target discrete perturbation
            if iSim == round(options.perturbation.t0/options.dt)+1 && length(options.perturbation.dx)==d
                xT(:,end+1) = xT(:,end) + options.perturbation.dx;
                XT = repmat(xT(:,end),1,nbSPoint);
                if options.plot %plotting options
                    subplot_results('t',sp,x,xT);
                end
            else
                xT(:,end+1) = xT(:,end);
            end
        case 'rdp' %applying robot discrete perturbation
            for ff=1:N_pl
                if iSim == round(options.perturbation.t0/options.dt)+1 && length(options.perturbation.dx)==d
                    x{ff}(:,iSim+1,:) = x{ff}(:,iSim+1,:) + repmat(options.perturbation.dx,[1 1 nbSPoint]);
                end
            end
        case 'tcp' %applying target continuous perturbation
            if iSim >= round(options.perturbation.t0/options.dt)+1 && iSim <= round(options.perturbation.tf/options.dt) && length(options.perturbation.dx)==d
                xT(:,end+1) = xT(:,end) + options.perturbation.dx*options.dt;
                XT = repmat(xT(:,end),1,nbSPoint);
                if options.plot %plotting options
                    subplot_results('t',sp,x,xT);
                end
            elseintersection_sf
                xT(:,end+1) = xT(:,end);
            end
        case 'rcp' %applying robot continuous perturbation
            for ff=1:N_pl
                if iSim >= round(options.perturbation.t0/options.dt)+1 && iSim <= round(options.perturbation.tf/options.dt) && length(options.perturbation.dx)==d
                    x{ff}(:,iSim+1,:) = x{ff}(:,iSim+1,:) + repmat(options.perturbation.dx,[1 1 nbSPoint])*options.dt;
                end
            end
    end
    
    % plotting the result
    if options.plot
        for ff=1:N_pl
            if d>1
                subplot_results('u',sp{ff},x{ff},xT);
            else
                subplot_results('u',sp{ff},[x{ff}(1:end-1);xd{ff}],[xT;0]);
            end
        end
    end
    
    xd_3last=[];
    for ff=1:N_pl
        xd_3last{ff} = xd{ff}(:,max([1 iSim-3]):iSim,:);
        xd_3last{ff}(isnan(xd_3last{ff})) = 0;
    end
    

    % Check collision
    if obs_bool 
        for ff=1:N_pl
            [coll, ~,~] = obs_check_collision(obs, squeeze(x{ff}(1,end,:)), squeeze(x{ff}(2,end,:)));
            if sum(coll)
                subplot(sp{ff}.fig)
                coll_list = (1:size(x{1},3));
                coll_list = coll_list(coll);
                for ix = coll_list
                    
                    plot(x{ff}(1,end,ix), x{ff}(2,end,ix), 'kd', 'LineWidth', 2); hold on;
                    warning('Collision detected at x=[%f2.3, %f2.3]', x{ff}(1,end,ix),x{ff}(2,end,ix))
                end
            end
        end
    end
    
    if options.saveAnimation
        if isfield(options,'x_range')
            xlim(options.x_range)
        end
        if isfield(options,'y_range')
            ylim(options.y_range)
        end
        writeVideo(vidObj, getframe(gca));
    end
    
    
    %Checking the convergence
    for ff = 1:N_pl
        if all(all(all(abs(xd_3last{ff})<options.tol))) || iSim>options.i_max-2
            if options.plot
                subplot_results('f',sp{ff},x{ff},xT);
            end
            iSim=iSim+1;
    %         xd(:,i,:)=reshape(fn_handle(squeeze(x(:,i,:))-XT),[d 1 nbSPoint]);
            x{ff}(:,end,:) = [];
            t(end) = [];
            fprintf('Number of Iterations: %1.0f\n',iSim)
            tmp='';
            for j=1:d
                tmp=[tmp ' %1.4f ;'];
            end
            tmp=tmp(2:end-2);
            fprintf('Final Time: %1.2f (sec)\n',t(1,end,1))
            fprintf(['Final Point (left): [' tmp ']\n'],squeeze(x{1}(:,end,:)))
            fprintf(['Final Points (right): [' tmp ']\n'],squeeze(x{2}(:,end,:)))
            fprintf(['Target Position: [' tmp ']\n'],xT(:,end))
            fprintf('## #####################################################\n\n\n')

            if iSim>options.i_max-2
                fprintf('Simulation stopped since it reaches the maximum number of allowed iterations i_max = %1.0f\n',iSim)
                fprintf('Exiting without convergence!!! Increase the parameter ''options.i_max'' to handle this error.\n')
            end
            if options.saveAnimation
                close(vidObj);
            end
            
            convergenceReached = true;
        else
            break
        end
    end
    if convergenceReached
        break
    end
    iSim=iSim+1;
end