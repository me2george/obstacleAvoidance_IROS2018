%function Tutorial_Obstacle_Avoidance_2018
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Copyright (c) 2018 S. Lukas Huber, LASA Lab, EPFL,         %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
warning('off','all') % turn warnings offjr
%% preparing the obstacle avoidance module
%adding the obstacle avoidance folder to the MATLAB path directories
if isempty(regexp(path,['lib_obstacle_avoidance' pathsep], 'once'))
    addpath([pwd, '/lib_obstacle_avoidance']);
end
if isempty(regexp(path,['DynamicalSystems' pathsep], 'once'))
    addpath([pwd, '/DynamicalSystems']);
end
if isempty(regexp(path,['lib_simulation_tools' pathsep], 'once'))
    addpath([pwd, '/lib_simulation_tools']);
end


%% initial comments
clc
disp('In this tutorial, we show examples of using the proposed obstacle')
disp('avoidance algorithm with different dynamical systems (DS). Throughout')
disp('this demo, both the DS and the obstacle equations are given. The file')
disp('''obs_modulation_ellipsoid.m'' in the ''lib_obstacle_avoidance'' folder,')
disp('is the only file that is necessary to perform the obstacle avoidance.')
disp('All other files are just needed for the illustrative purpose.')
disp('press any key to continue ...')
pause

%% first demo
disp(' ')
disp(' ')
disp('In the first demo, we consider a globally asymptotically stable DS,')
disp('which is defined by:')
disp('  xd = -x ;')
disp('This DS has a unique attractor at the origin.')
disp('press any key to draw the streamlines of this DS ...')
pause

fn_handle = @(x) linearStableDS(x); %defining the function handle
x0 = [-18*ones(1,15);linspace(-10,10,15)]; %set of initial points
% A set of parameters that should be defined for the simulation
opt_sim.dt = 0.02; %integration time steps

opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.model = 1; %first order ordinary differential equation
opt_sim.obstacle = []; %no obstacle is defined
fig(1) = figure('name','First demo: Streamlines of the original DS','position',[200 200 1100 500]);
opt_sim.figure = fig(1);
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp(' ')
disp(' ')
disp('Now, let us evaluate the system behavior in the presence of a single')
disp('obstacle, that is defind by:')
disp('obs{1}.a  = [3 3]')
disp('obs{1}.p  = [1 1')
disp('obs{1}.x0 = [-8;0]')
disp('obs{1}.sf = [1.2]')
disp('obs{1}.th_r = 0')
disp('press any key to draw the streamlines in the presence of a circular obstacle ...')
pause

clear obs;
obs{1}.a = [3; 3];
obs{1}.p = [1;1];
obs{1}.x0 = [-8;0];
obs{1}.sf = [1.2];
obs{1}.th_r = 0*pi/180;
opt_sim.obstacle = obs;
fig(2) = figure('name','First demo: Streamlines of the modulated DS (circular)','position',[200 200 1100 500]);
opt_sim.figure = fig(2);
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause

disp(' ')
disp(' ')
disp('Now, let us evaluate the system behavior in the presence of a non-circualr obstacle')
disp('obstacle, that is defind by:')
disp('obs{1}.a  = [1.2 3]')
disp('obs{1}.p  = [2 1')
disp('obs{1}.x0 = [-8;0]')
disp('obs{1}.sf = [1.2]')
disp('obs{1}.th_r = 0')
disp('press any key to draw the streamlines in the presence of the obstacle ...')
pause

clear obs;
obs{1}.a = [1.2; 3];
obs{1}.p = [2;1];
obs{1}.x0 = [-8;0];
obs{1}.sf = [1.2];
obs{1}.th_r = 0*pi/180;
opt_sim.obstacle = obs;
fig(2) = figure('name','First demo: Streamlines of the modulated DS','position',[200 200 1100 500]);
opt_sim.figure = fig(2);
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause

% adding more obstacles
disp(' ')
disp(' ')
disp('Now let us add two more obstacles to the previous example. We use the')
disp('same geometry for the new obstacles, but we place them in different')
disp('positions, and also rotate them with different angles.')
disp('The second obstacle:')
disp('  obs{2} = obs{1}')
disp('  obs{2}.x0 = [-12;3]')
disp('  obs{2}.th_r = 90*pi/180')
disp('The third obstacle:')
disp('  obs{3} = obs{1}')
disp('  obs{3}.x0 = [-12;-3]')
disp('  obs{3}.th_r = -90*pi/180')

obs{1}.x0 = [-4 ;0];
obs{2} = obs{1};
obs{2}.x0 = [-12;3];
obs{2}.th_r = 90*pi/180;
obs{3} = obs{1};
obs{3}.x0 = [-12;-3];
obs{3}.th_r = -90*pi/180;
disp('press any key to draw the streamlines in the presence of three obstacles ...')
pause

opt_sim.obstacle = obs;
fig(3) = figure('name','1st demo: Multiple obstacle avoidance','position',[200 300 1100 500]);
opt_sim.figure = fig(3); 
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause
close all;


%% Second demo -- obstacles are intersecting
disp(' ')
disp(' ')
disp('In the second demo, intersecting obstacles are observed')
disp('the shape of the obstacles varied, and furthermore')
disp('they are placed such that they intersect with each other.')
disp('First lets observe an intersection, where the concave region')
disp('is facing towards the incoming initital DS.')

obs = [];
obs{1}.a = [1.2; 3.5];
obs{1}.p = [1;1];
obs{1}.x0 = [-8;-2.5];
obs{1}.sf = [1.2];
obs{1}.th_r = 110*pi/180;

obs{2}.a = [1.2;3.5];
obs{2}.p = [1;1];
obs{2}.x0 = [-6;1];
obs{2}.sf = [1.2];
obs{2}.th_r = 30*pi/180;

disp('press any key to draw the streamlines in the presence of a concave obstacle ...')

% pause
opt_sim.obstacle = obs;
fig(3) = figure('name','1st demo: Concave regions (1)','position',[200 300 1100 500]);
opt_sim.figure = fig(3); 
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause


disp(' ')
disp(' ')
disp('Let us have the same obstacles, but intersecting in a different ')
disp('namely their concave region is now facing down, perendicular to the ')
disp('stream flow. ')

obs{1}.x0 = [-10;0.5];
obs{1}.th_r = -40*pi/180;

obs{2}.x0 = [-6;1];
obs{2}.th_r = 30*pi/180;

disp('press any key to draw the streamlines in the presence of a concave obstacle ...')

% pause
opt_sim.obstacle = obs;
fig(1) = figure('name','2nd demo: Concave regions (2)','position',[200 300 1100 500]);
opt_sim.figure = fig(1); 
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause
close all;


%% Moving obstacles
disp(' ')
disp(' ')
disp('In the third demo, moving obstacles that drive the robot')
disp(' we first observe a linear motion along the x2 axis with a ')
disp(' velocity of -5')

obs = [];
obs{1}.a = [1.2; 3.5];
obs{1}.p = [1;1];
obs{1}.x0 = [-8;4];
obs{1}.sf = [1.2];
obs{1}.th_r = 60*pi/180;
obs{1}.perturbation.t0 = 0;
obs{1}.perturbation.tf = 3;
obs{1}.perturbation.dx = [0;-5];  

disp('press any key to draw the streamlines in the presence of a moving obstacles ...')

% pause
opt_sim.obstacle = obs;
fig(2) = figure('name','3rd demo: Obstacle with linear velocity','position',[200 300 1100 500]);
opt_sim.figure = fig(2); 
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause

disp(' ')
disp(' ')
disp('We further assess the behaviour in the presence of a angular ')
disp('velcity. The geometry is the same as in the previous case, and the ')
disp('angular velocity is 4. ')

obs{1}.x0 = [-8;0];
obs{1}.perturbation.w = 4;  
obs{1}.perturbation.dx = [0;0];

disp('press any key to draw the streamlines in the presence of three obstacles ...')
pause

opt_sim.obstacle = obs;
fig(3) = figure('name','3rd demo: Obstacle with angular velocity','position',[200 300 1100 500]);
opt_sim.figure = fig(3); 
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause


disp(' ')
disp(' ')
disp('In following demo, we observe two moving obstacles. The first ')
disp('is moving in such a way, that it collides with the second one and ')
disp('temporarily forms a concave region. The center of the obstacles ')
disp('circle has to dynamically adapt to compensate for this.')

obs{1}.x0 = [-8;6];
obs{1}.perturbation.w = 0;  
obs{1}.perturbation.dx = [0;-5];

obs{2} = obs{1};
obs{2}.x0 = [-8;-2];
obs{2}.th_r = -80*pi/180;
obs{2}.perturbation.dx = [0;-0];

disp('press any key to draw the streamlines in the presence of three obstacles ...')

% pause
opt_sim.obstacle = obs;
fig(3) = figure('name','3rd demo: Multiple, moving obstacles','position',[200 300 1100 500]);
opt_sim.figure = fig(3); 
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
pause
close all;

%% Fourth demo - nonlinear vector field
disp(' ')
disp(' ')
disp('In the fourth demo, we consider a nonlinear system with a stable limit cycle that drive the robot')
disp('motion:')
disp('  xd = x(2,:);')
disp('  xd(2,:) = -x(1,:) + 0.9*(1 - x(1,:).^2).*x(2,:);')
disp('press any key to draw the streamlines of this DS ...')
%pause
fn_handle = @(x) limitcycle_DS(x); %defining the function handle
x0 = [zeros(1,10);linspace(0,3,10)]; %set of initial points
% A set of parameters that should bedefined for the simulation
opt_sim.dt = 0.01; %integration time steps
opt_sim.i_max = 1000; %maximum number of iterations
opt_sim.tol = 0.05; %convergence tolerance
opt_sim.plot = true; %enabling the animation
opt_sim.obstacle = []; %no obstacle is defined

fig(1) = figure('name','4th demo: Limit Cycle','position',[200 300 1100 500]);
opt_sim.figure = fig(1);
Simulation_comparison(x0,[],fn_handle,opt_sim);

%% pause
disp(' ')
disp(' ')
disp('For simplicity, we use the same geometry as the previous demo, but')
disp('with different positions. In this example, we consider two obstacles.')
disp('press any key to draw the streamlines in the presence of the obstacles ...')
%pause
obs{1}.a = [0.7;0.6];
obs{1}.x0 = [-2.5;-1];
obs{1}.perturbation.dx = [-0;0];

obs{2}.a = [1;0.5];
obs{2}.x0 = [2;1];
obs{2}.perturbation.dx = [-0;0];

opt_sim.obstacle = obs;
fig(2) = figure('name','First demo: Multiple obstacle avoidance','position',[200 300 1100 500]);
opt_sim.figure = fig(2);
Simulation_comparison(x0,[],fn_handle,opt_sim);
disp('press any key to continue ...')
