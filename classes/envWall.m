classdef envWall < envObject
    %"Describing and Analyzing Mechanical Contact for Continuum Robots Using a Shooting-Based Cosserat Rod Implementation"
    % Class of type envObject for defining a wall
    % Darstellung über Normalform. wall_normal zeigt in die Wand 

    properties
        %Appendix: "
        wall_normal %normal vector pointing inside wall w_wall
        wall_point  %Point on wall x_wall
    end
    
    methods
        function obj = envWall(wall_normal, wall_point)
            
            obj@envObject();
           
            obj.wall_normal = wall_normal;
            obj.wall_point = wall_point;
            
        end

        function [gap, w_lever] = get_gap(obj,y_seg,segment_radius)
            % computes gap values and lever between the object and a given node 
            % assumes disk with segment_radius around node
            % y_seg: state of robot node
            r_seg = y_seg(1:3); % node position
            h_seg = y_seg(4:7); % orientation (quaternion)
            R_seg = quat2rot(h_seg); % orientation matrix
            w_dist = obj.wall_normal;
            
            %project wall normal onto plane containing disk 
            %eq. 11 
            w_lever = w_dist - w_dist'*R_seg(:,3) * R_seg(:,3); % eq. 
            w_lever = w_lever/norm(w_lever); %normalize
            w_lever = w_lever*segment_radius;
            
            potential_contact_point = r_seg+w_lever; %xc
            gap = (obj.wall_point-potential_contact_point)'*obj.wall_normal;
            
           
        end

        function v_force_dir_norm = get_force_direction_norm(obj,y_seg,w_lever)
            v_force_dir_norm = -obj.wall_normal;
        end
        function v_force_dir_fric = get_force_direction_fric(obj,y_seg,prev_state,v_force_dir_norm,w_lever)
            %compute direction of tangential contact force (friction)
            % y_seg: state of robot node
            % prev_state: state of robot node in previous simulation step
            r_seg = y_seg(1:3);
            h_seg = y_seg(4:7);
            R_seg = quat2rot(h_seg);
            r_seg_old = prev_state(1:3);
            h_seg_old = prev_state(4:7);
            R_seg_old = quat2rot(h_seg_old);

            contact_point = r_seg+w_lever;

            segment_radius = norm(w_lever);
            
            %eq. 15:
            R_lever = [ w_lever/norm(w_lever) cross(R_seg(:,3),w_lever)/norm(cross(R_seg(:,3),w_lever)) R_seg(:,3)];
           
            %eq. 16:
            R_rot =  R_lever'*R_seg; % Kontaktpunkt im lokalen Koordinatensystem (Transformation matrix (rotation))
            RR_old = (R_rot*R_seg_old')'; % Rotationsmatrix für Kontaktpunkt im vorherigen Schritt im lokalen Koordinatensystem
            
            %eq. 17:
            w_lever_old = segment_radius * RR_old(:,1);


            %eq. 14:
            contact_point_old = r_seg_old +  w_lever_old; %point of contact in previous simulation step
            
            % eq. 13: direction of motion of contact point
            w_motion = (contact_point-contact_point_old);

            % eq. 18: project onto object's surface (normalized in paper, but thats wrong)
            v_force_dir_fric = w_motion - w_motion'*v_force_dir_norm * v_force_dir_norm;           
        end
    
        
    end
end