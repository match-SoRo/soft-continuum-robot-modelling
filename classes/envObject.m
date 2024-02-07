classdef envObject	
    % Class envObject is Superclass of envWall, envSphere, envCylinder ...
    
    properties
        penalty_stiffness   %penalty stiffness paramter
        p_regt              %regularization parameter frictional contact (threshhold)
        p_regn              %regularization parameter normal contact force
        fric_coeff          %friction coefficient (Coulomb)
    end
    
    methods
        function obj = envObject()
            %initilialisiere Kontaktparameter
            step = 250;
            obj.fric_coeff          = 0.66;
            obj.penalty_stiffness   = 2*5*1/step*2500000;%*2
            obj.p_regt              = 2.5e-5;
            obj.p_regn              = 5e-3;   
        end
        function fric_coeff = get_fric(obj)
            fric_coeff = obj.fric_coeff;
        end
    end
end

