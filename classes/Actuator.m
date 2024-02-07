classdef Actuator < Segment
        
    % Class of type Segment for defining a Actuator-section
    
    properties
        chamber_radius
        chamber_area
        chamber_pos
        chamber_distance
        fiber_radius
        shear_factor
        axial_factor
                
    end
    
    methods
        function obj = Actuator(segment_radius, segment_length, material, chamber_radius, fiber_radius, midchannel_radius, chamber_distance, n_nodes_seg, ide_params)
            obj@Segment(segment_radius, segment_length, material, n_nodes_seg);
            
            obj.chamber_area = pi*chamber_radius^2;
            obj.segment_area = pi*segment_radius^2 - 3*obj.chamber_area - pi*midchannel_radius^2;

            obj.chamber_radius = chamber_radius;
            obj.fiber_radius = fiber_radius;
            obj.shear_factor = 0;
            obj.axial_factor = 1;
            % Positions of the chambers
            obj.chamber_pos = zeros(3,3);
            obj.chamber_pos(:,1) = chamber_distance*[-1; 0; 0];
            obj.chamber_pos(:,2) = chamber_distance*[-cos(4*pi/3); sin(4*pi/3); 0];
            obj.chamber_pos(:,3) = chamber_distance*[-cos(2*pi/3); sin(2*pi/3); 0];
            
            % Second moment of area of channel/chamber [m^4]
            chamber_inertia = pi*chamber_radius^4/4;       
            midchannel_inertia = pi*midchannel_radius^4/4;
            
            % Second moment of area of total segment [m^4]
            Ixx = pi*segment_radius^4/4 - midchannel_inertia - 3*chamber_inertia - ((obj.chamber_pos(2,1)^2)*obj.chamber_area) - ((obj.chamber_pos(2,2)^2)*obj.chamber_area) - ((obj.chamber_pos(2,3)^2)*obj.chamber_area);
            Iyy = pi*segment_radius^4/4 - midchannel_inertia - 3*chamber_inertia - ((obj.chamber_pos(1,1)^2)*obj.chamber_area) - ((obj.chamber_pos(1,2)^2)*obj.chamber_area) - ((obj.chamber_pos(1,3)^2)*obj.chamber_area);
            J1 = Ixx + Iyy;
            
            obj.inertia = diag([Ixx Iyy J1]);
            
            obj.material = obj.material.calc_K_se_K_bt(obj.segment_area, obj.inertia);
            
            obj.is_actuated = true;
            
        end
        function obj = update_shear_factor(obj,shear_factor)
            obj.shear_factor = shear_factor;
        end

    end
end

