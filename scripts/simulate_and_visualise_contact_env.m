% example script for simulation and identification
clear all
close all 
clc

%save ooptions
cos_data_name = 'data_cos_cyl.mat';
fig_name = 'fig_cyl_fs2';

E = [32e4 1.7348e5 32e4]; %0.68*        % Youngs Modulus (per segment/Section)
G = E./(2*(1+0.5));         % Schubmodul

% Define Material for each segment:
% arguments: material_name, material_type, material_law, E, G, rho
% material_types: 1=lin, 2=ANN, 3=nonlin
m_cap_base  = Material('DragonSkin', 1, E(1), G(1), 1080);
m_tube      = Material('Ecoflex', 1, E(2), G(2), 1070);
m_cap_tip   = Material('DragonSkin', 1, E(3), G(3), 1080);
m_base      = Material('art', 1 ,200e9, 200e9, 842.1635);


% Define Loads (pressure samples)
gravity = [0; 0; 9.81];
tip_mass = 0;

n_ext = zeros(3,1); %external forces
m_ext = zeros(3,1); %external moments
s_n_ext = 1; %position of external force along system's length 
s_m_ext = 1; %position of external moment along system's length

step = 500; %pressure increase per simulation step
roundF = 4;

p_fak = [1; 0; 0];
p_max = 100000;
[P, robot_loads, robot_load] = pressureProfile(step,p_max,p_fak,roundF,gravity, tip_mass, n_ext, m_ext, s_n_ext, s_m_ext);

Y_sim = cell(1,size(P,2));

% Define 3 segments of type Cap, Actuator, Cap
% args for Cap: segment_radius, segment_length, material, robot_loads, midchannel_radius, n_nodes_seg
% args for Actuator: segment_radius, segment_length, material, robot_loads,
% chamber_radius, midchannel_radius, chamber_distance, n_nodes_seg
offset = 0.023;
offsetTip = 0.002;

segments{1} = Cap(0.0211, offset, m_base, 0, 2);
segments{2} = Cap(0.0211, 0.01, m_cap_base, 0.0035, 3);
segments{3} = Actuator(0.0211, 0.110, m_tube, 0.00515, 0.0067, 0.0035, 0.012, 40);

segments{3}.shear_factor = 6.5401e-5; %see "Investigation of Lateral Compression Effects in Fiber Reinforced Soft Pneumatic Actuators"
segments{4} = Cap(0.0211, 0.01, m_cap_tip, 0.0035, 3);
segments{5} = Cap(0.0211, offsetTip, m_base, 0, 2);


iObj = 1;
visW = 0;
visS = 0;
visC = 1;
objects = {};
objects{iObj} = envWall([1; 0; 0.]/norm([1; 0.; 0.]),[0.5+0.0005+0.0211; -0.9*0.0211*0; 0.1*0.155]); iObj = iObj+1;
% objects{iObj} = envCylinder(0.03,[0.5+0.03+0.0211+0.0005; 0; 0.069+offset],[0;1;0]/norm([0 1 0]));
objects{iObj} = envCylinder(0.03,[54.164e-3; -18.809e-3; 0.03+0.069+offset],[1;1;0]/norm([1 1 0]));


% robot = Robot(segments, robot_loads);
robot = Robot(segments, {robot_load});
L = robot.robot_length;

optimizer = 'levenberg-marquardt';


% Define the Boundary Conditions for simulation
bc = BoundaryCondition;
bc = bc.set_base_bc('r', [0;0;0]);
bc = bc.set_base_bc('h', [1;0;0;0]);
bc = bc.set_tip_bc('n', [0;0;0]);
bc = bc.set_tip_bc('m', [0;0;0]);

% comment in to generate mex files (simulate_robot_mex)
% generate_code(robot, objects, bc, optimizer, 'sim', 'mex')


n_optim_params_sim = robot.n_states - bc.get_nr_of_init_values;
optimized_params = zeros(n_optim_params_sim,1);

prev_state = zeros(13,robot.n_nodes);
numSamples = max(size(robot_loads));
kk = 0;
for i_sample = 1:numSamples
       
    % simulate
    tic;
    prev_ide_params = optimized_params; 
    robot_load_s{1} = robot_loads{i_sample};
    robot = Robot(segments, robot_load_s);
    
    [y_sim, residual, optimized_params, exitflag(i_sample),contactStruct] = simulate_robot(robot, objects, 1, bc, optimizer, prev_ide_params, prev_state);
%     [y_sim, residual, optimized_params, exitflag(i_sample),contactStruct] = simulate_robot_mex(robot, objects, 1, bc, optimizer, prev_ide_params, prev_state);

    prev_state_old = prev_state;
    prev_state = y_sim; 
    T(i_sample) = toc;
    
    Y_sim{i_sample} = y_sim;
    
    y_sim = y_sim';
    
    i_sample
    pause(0.001)
    
    %visualize every 50th simulation step
    if mod(i_sample,50)==0 || i_sample==1
        kk=kk+1;
        close
        fig = figure('Units','points','Position',[100 100 505.89 395],'visible','on');
        
        ind = 2; %plot every ind-th node
        plot_bbc(y_sim,robot,ind)
        plot_obj
 
        set(gca,'zdir','reverse')
        set(gca,'ydir','reverse')
        ax = gca;
        limits = ax(1).ZLim;

        zlim([offset 0.17]);
        if visC == 1
            xlim([-0.0225 0.0814]); %Cylinder
        elseif visW == 1
            xlim([-0.1036/2 0.1036/2]) %Wall
        end
        hold off
        F(kk)=getframe;
        set(0, 'DefaultFigureRenderer', 'painters');
        camproj('perspective')
        view([0 1])
        fig_name_spec = [fig_name,'_',num2str(i_sample),'.fig'];
        savefig(fig_name_spec)
    end
end
Tsim = sum(T);

%save data
data_standard_struct.time = zeros(numSamples,1);
data_standard_struct.p_set = P';
data_standard_struct.p_is = P';
for i_sample = 1:numSamples
    y_sim = Y_sim{i_sample}';
    data_standard_struct.x_is(i_sample,:) = y_sim(end,1:3);
    data_standard_struct.h_is(i_sample,:) = y_sim(end,1:4);
    data_standard_struct.R_is(:,:,i_sample) = quat2rot(y_sim(end,4:7)');
    data_standard_struct.force_torque(i_sample,:) = y_sim(1,8:13); 
    data_standard_struct.centerline(:,:,i_sample) = y_sim(:,1:3);
    data_standard_struct.full_results(:,:,i_sample) = y_sim;
end
data_standard_struct.gravity = gravity;
save(cos_data_name,'data_standard_struct');
