function [y, residual, optimized_params, exitFlag, contactStruct] = simulate_robot(robot, objects, i_sample, bc, optimizer, prev_ide_params, prev_state) %#codegen
%SIMULATION optimizes the objective function get_residual
% which computes the states of the Robot
exitFlag = 0;

% i_n_ext_L = robot.robot_loads{i_sample}.s_n_ext == robot.segments{end}.s_vec(end);
% i_m_ext_L = robot.robot_loads{i_sample}.s_m_ext == robot.segments{end}.s_vec(end);

get_residual_sim_h = @(optim_params) get_residual_sim(optim_params, robot, objects, i_sample, bc, prev_state);


options = optimoptions('fsolve','Algorithm', optimizer,...
    'FunctionTolerance',1e-6,'MaxFunctionEvaluations',12000,'MaxIterations',12000,...
    'StepTolerance',1e-16,'CheckGradients',false);%,'FiniteDifferenceType','central');%e-13
options.InitDamping = 1e-2;

% prev_ide_params: Identified state at base found via shooting-> initial
% search value for new sim step
if any(prev_ide_params ~= 0) && max(size(prev_ide_params)) == robot.n_states - bc.get_nr_of_init_values
    initial_guess = prev_ide_params;
else
    initial_guess = set_initial_guess(bc, robot);
end

if isempty(initial_guess)
    optimized_params = 0;
else
    [optimized_params, ~, exitFlag, output] = fsolve(get_residual_sim_h, initial_guess, options);
end

[residual, y, contactStruct] = get_residual_sim_h(optimized_params);
end





















































































































































