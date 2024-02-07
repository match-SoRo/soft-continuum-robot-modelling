classdef BoundaryCondition
    %BOUNDARY_CONDITION Summary of this class goes here
    %   Detailed explanation goes here
    
%     properties(SetAccess = private)
    properties
        
        % initial condition for the integration
        r0 % position (dim 3x1)
        h0 % orientation as quaternion (dim 4x1, unitless)
        n0 % force (dim 3x1)
        m0 % moment (dim 3x1)
        
        % end condition for calc of residual
        rL % position (dim 3x1)
        hL % orientation as quaternion (dim 4x1, unitless)
        nL % force (dim 3x1)
        mL % moment (dim 3x1)
        
        bc_list % list of all used boundary conditions
        
        
    end
    
    methods
        function obj = BoundaryCondition
            %BOUNDARY_CONDITION Construct an instance of this class
            
            obj.bc_list = {};
            % needs to be preallocated for codegen
            obj.r0 = zeros(3,1);
            obj.rL = zeros(3,1);
            obj.h0 = zeros(4,1);
            obj.hL = zeros(4,1);
            obj.m0 = zeros(3,1);
            obj.mL = zeros(3,1);
            obj.n0 = zeros(3,1);
            obj.nL = zeros(3,1);
        end
        
        function obj = set_base_bc(obj, state_var, state_value)
            %SET_BASE_BC sets boundary condition state_var at the base to state value.
            % This will be used as initial value (b2t)
            % state var can be 'r', 'h', 'n' or 'm'
            switch state_var
                case 'r'
                    obj.r0 = state_value;
                    if ~any(strcmp(obj.bc_list, 'r0'))
                        obj.bc_list{numel(obj.bc_list)+1} = 'r0';
                    end
                case 'h'
                    obj.h0 = state_value;
                    if ~any(strcmp(obj.bc_list, 'h0'))
                        obj.bc_list{numel(obj.bc_list)+1} = 'h0';
                    end
                case 'n'
                    obj.n0 = state_value;
                    if ~any(strcmp(obj.bc_list, 'n0'))
                        obj.bc_list{numel(obj.bc_list)+1} = 'n0';
                    end
                case 'm'
                    obj.m0 = state_value;
                    if ~any(strcmp(obj.bc_list, 'm0'))
                        obj.bc_list{numel(obj.bc_list)+1} = 'm0';
                    end
                otherwise
                    disp("enter correct value");

            end
        end
        
        function obj = set_tip_bc(obj, state_var, state_value)
            %SET_TIP_BC sets boundary condition state_var at the tip to state_value.
            % This will be used to calc residuum (b2t)
            % state var can be 'r', 'h', 'n' or 'm'
            switch state_var
                case 'r'
                        obj.rL = state_value;
                        if ~any(strcmp(obj.bc_list, 'rL'))
                            obj.bc_list{numel(obj.bc_list)+1} = 'rL';
                        end
                case 'h'
                        obj.hL = state_value;
                        if ~any(strcmp(obj.bc_list, 'hL'))
                            obj.bc_list{numel(obj.bc_list)+1} = 'hL';
                        end
                case 'n'
                        obj.nL = state_value;
                        if ~any(strcmp(obj.bc_list, 'nL'))
                            obj.bc_list{numel(obj.bc_list)+1} = 'nL';
                        end
                case 'm'
                        obj.mL = state_value;
                        if ~any(strcmp(obj.bc_list, 'mL'))
                            obj.bc_list{numel(obj.bc_list)+1} = 'mL';
                        end                  
                otherwise
                    disp("enter correct value");
            end
        end
        
               
        function res = get_nr_of_init_values(obj)
            res = 0;
            if obj.has_value('r0')
                res = res+3;
            end
            if obj.has_value('h0')
                res = res+4;
            end
            if obj.has_value('n0')
                res = res+3;
            end
            if obj.has_value('m0')
                res = res+3;
            end
        end
        
        
        function res = has_value(obj, value)
            res = any(strcmp(obj.bc_list, value));
        end
        
        
    end
end

