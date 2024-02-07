classdef Robot
    % contains all segments (cap or Actuator) as a cell structur 'segments'
    
    properties
        segments % cell array of all segments
        n_segments
        n_states % number of states in ode system
        n_nodes % number of dicretizations
        robot_length
%         n_ide_params % total number of identification parameters
        robot_loads % cell array of Load objects
    end
    
    methods(Access = public)
        function obj = Robot(segments, robot_loads)

            obj.n_nodes = 0;
            obj.robot_length = 0;
%             obj.n_ide_params = 0;  
            obj.segments = segments;
            obj.n_segments = length(obj.segments);
            
            s = 0;
            for i_seg = 1:obj.n_segments
                obj.robot_length = obj.robot_length + obj.segments{i_seg}.segment_length;
                obj.n_nodes = obj.n_nodes + obj.segments{i_seg}.n_nodes_seg;
%                 obj.n_ide_params = obj.n_ide_params + sum(obj.segments{i_seg}.ide_params);

                sL_last_seg = s;
                for i_node_seg = 1:obj.segments{i_seg}.n_nodes_seg
                    s = sL_last_seg + (i_node_seg-1)*obj.segments{i_seg}.stepsize;
                    obj.segments{i_seg}.s_vec(i_node_seg) = s;
                end
             
            end
            obj.n_states = 13;
   
            for i_sample = 1:max(size(robot_loads))                
                % create zero vectors in 'pressure' for not actuated segments
                pressure = zeros(3,obj.n_segments);
                for i_seg = 1:obj.n_segments
                    if obj.segments{i_seg}.is_actuated && ~isempty(robot_loads{i_sample}.pressure) 
                        pressure(:,i_seg) = robot_loads{i_sample}.pressure(:,1);
                        robot_loads{i_sample}.pressure = robot_loads{i_sample}.pressure(:,2:end);
                    end
                end
                robot_loads{i_sample}.pressure = pressure;                
                
            end
            obj.robot_loads = robot_loads;
            
        end
        
    end
    
end