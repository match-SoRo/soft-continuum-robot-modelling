classdef Segment	
    % Class Segment is Superclass of Cap or Tupe
    % Contains geometrical and material Properties
    % ide_params: contains a list of all parameters 
    % to be identified
    
    properties
        segment_radius
        segment_length
        segment_area
        inertia % Second moment of area [mm^4]
        material % object of class material containing all material properties
        stepsize % stepsize for RK integration
        s_vec % vector containing positions of nodes along whole s
        n_nodes_seg

        is_actuated
        u_star
        v_star
        
    end
    
    methods
        function obj = Segment(segment_radius, segment_length, material, n_nodes_seg)
            obj.segment_radius = segment_radius;
            obj.segment_length = segment_length;
            obj.material = material;
            obj.n_nodes_seg = n_nodes_seg;
            obj.stepsize = (1 / (n_nodes_seg-1)) * segment_length;
            obj.s_vec = zeros(1,n_nodes_seg);           
            obj.is_actuated = false;
            obj.u_star = [0;0;0];
            obj.v_star = [0;0;1];

        end
        
        function obj = set_uv_star(obj,u_star,v_star)
            obj.u_star = u_star;
            obj.v_star = v_star;
        end

    end
end

