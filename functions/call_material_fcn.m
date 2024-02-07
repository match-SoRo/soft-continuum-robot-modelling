function [par_se, par_bt] = call_material_fcn(segment, current_state, s, pressure, mat_law_type)
%CALL_MATERIAL_FCN evaluates the material law
% accordingly to the specified type
% Describe mat_law_type
% 1 - lin
% 2 - ANN
% 3 - nonlin

par_se = zeros(3,1);
par_bt = zeros(3,1);

switch mat_law_type
    
    case 1 % lin
        par_se = diag(segment.material.K_se);
        par_bt = diag(segment.material.K_bt);
        
    case 2 % ANN
        % TODO
        par_se = ones(3,1);
        par_bt = ones(3,1);                       
    case 3
        % TODO
        par_se = ones(3,1);
        par_bt = ones(3,1);

end
end

