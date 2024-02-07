classdef Cap < Segment
    
    % Class of type Segment for defining a cap-section
    
    properties
        
    end
    
    methods
        function obj = Cap(segment_radius, segment_length, material, midchannel_radius, n_nodes_seg)
            
            obj@Segment(segment_radius, segment_length, material, n_nodes_seg);
                        
            obj.segment_area = pi*segment_radius^2 - pi*midchannel_radius^2;
            
            midchannel_inertia = pi*midchannel_radius^4/4;     
            
            Ixx = pi*segment_radius^4/4 - midchannel_inertia;
            Iyy = pi*segment_radius^4/4 - midchannel_inertia;
            J1 = Ixx + Iyy;
            
            obj.inertia = diag([Ixx Iyy J1]);
            obj.material = obj.material.calc_K_se_K_bt(obj.segment_area, obj.inertia);
            
            obj.is_actuated = false;
            
        end
        

    end
end

