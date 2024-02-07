function generate_code(robot, objects, bc, optimizer, sim_or_ide, output_lang)
%generate_code compiles 'simulate_robot' or 'identify_parameters'
% either as mex files or cpp source code
% in order to generate the code some hardcoded insertions are made to the functions simulate_robot,
% get_residual_sim, get_residual_ide, identify_parameters,
% set_initial_guess during the compilition. 
%  
% sim_or_ide:   1=sim
%               2=ide
% output_lang:  1=mex
%               2=cpp              


cfg.EnableVariableSizing = true;
if output_lang == 'mex'
    cfg = coder.config('mex');
elseif output_lang ==  'cpp'
    cfg = coder.config('dll');
    cfg.TargetLang = 'C++';
end


if sim_or_ide == 'sim'
    
    opt_var_size = robot.n_states - bc.get_nr_of_init_values;
    inputs = {robot, objects, 1, bc, optimizer, zeros(opt_var_size, 1), zeros(13,robot.n_nodes)};
    
    insert_at = ["function [residual, y, contactStruct] = get_residual_sim(optim_params, robot, objects, i_sample, bc, prev_state)",
        "function [y, residual, optimized_params, exitFlag, contactStruct] = simulate_robot(robot, objects, i_sample, bc, optimizer, prev_ide_params, prev_state) %#codegen",
        "function [y, residual, optimized_params, exitFlag, contactStruct] = simulate_robot(robot, objects, i_sample, bc, optimizer, prev_ide_params, prev_state) %#codegen",
        "function [y, residual, optimized_params, exitFlag, contactStruct] = simulate_robot(robot, objects, i_sample, bc, optimizer, prev_ide_params, prev_state) %#codegen",];   
    insert_text = {['robot.n_segments = ',num2str(robot.n_segments),';'],
        ['coder.varsize(''initial_guess'', ', '[', num2str([opt_var_size 1]),'], [1 0]);'],
        ['optimizer = ''',optimizer,''';'],
        ['robot.n_segments = ',num2str(robot.n_segments),';']};
    function_names = {'functions/get_residual_sim.m',
        'functions/simulate_robot.m',
        'functions/simulate_robot.m',
        'functions/simulate_robot.m'};
    
        
    
    for i = 1:max(size(insert_at))
        insert_after(function_names{i}, insert_at(i), insert_text{i});
    end
    
    codegen -config cfg -args inputs simulate_robot
    if output_lang == 'mex'
        movefile(['simulate_robot','_mex.mexw64'],'functions','f');
    end
    
    for i = 1:max(size(insert_at))
        remove_string(function_names{i}, insert_text{i});
    end
    
    
else
    return
    
    
end


end


% Helper functions
function insert_after(file_name, insert_at, insert_text)

% takes file in file_name, creates a copy into codegen/tmp.m
% inserts 'isert_text' after the line in 'insert_at'

if ~isfile(file_name)
    error ('%s does not exist.', file_name);
end
% copyfile(file_name, 'codegen/tmp.m');
file_text = fileread(file_name);
file_text  = split(file_text, newline);
new_file_text = [];
for i = 1:max(size(file_text))
    new_file_text = [new_file_text, file_text{i}, newline];
    if strcmp(convertCharsToStrings(file_text{i}(1:end-1)), insert_at)
        new_file_text = [new_file_text, insert_text, newline];
    end
end

file_id = fopen(file_name, 'w');
fwrite(file_id, new_file_text);
fclose(file_id);

end

function remove_string(file_name, remove_text)

% removes line in file 'file_name' which matches 'remove_text'

if ~isfile(file_name)
    error ('%s does not exist.', file_name);
end
% copyfile(file_name, 'codegen/tmp.m');
file_text = fileread(file_name);
file_text  = split(file_text, newline);
new_file_text = [];
for i = 1:max(size(file_text))
    if ~strcmp(convertCharsToStrings(file_text{i}(1:end)), remove_text)
        new_file_text = [new_file_text, file_text{i}, newline];
    end
end
while new_file_text(end) == newline && new_file_text(end-1) == newline
    new_file_text = new_file_text(1:end-1);
end
file_id = fopen(file_name, 'w');
fwrite(file_id, new_file_text);
fclose(file_id);

end