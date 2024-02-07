classdef envCylinder < envObject
    
    % Class of type envObject for defining a sphere
    
    properties
        cylinder_radius
        cylinder_point
        cylinder_axis
    end
    
    methods
        function obj = envCylinder(cylinder_radius, cylinder_point, cylinder_axis)
            
            obj@envObject();
           
            obj.cylinder_radius = cylinder_radius;
            obj.cylinder_point = cylinder_point;
            obj.cylinder_axis = cylinder_axis;
            
        end

        function [gap, w_lever] = get_gap(obj,y_seg, segment_radius)
            % assumes disk with segment_radius around node  
            % y_seg state of robot node
            r_seg = y_seg(1:3);
            h_seg = y_seg(4:7);
            R_seg = quat2rot(h_seg);
    
            x_obj_closest = obj.cylinder_point+(r_seg-obj.cylinder_point)'*obj.cylinder_axis*obj.cylinder_axis;
            cylinder_seg_normal = (x_obj_closest-r_seg); %w_dist
            %project normal onto plane containing disk 
            cylinder_normal_pro = cylinder_seg_normal - dot(cylinder_seg_normal,R_seg(:,3))/(norm(R_seg(:,3))^2) * R_seg(:,3);
            cylinder_normal_pro = cylinder_normal_pro/norm(cylinder_normal_pro); %normalize 
            potential_contact_point = r_seg+cylinder_normal_pro*segment_radius;
            
            gap = norm(potential_contact_point-x_obj_closest)-obj.cylinder_radius;
            
            w_lever = cylinder_normal_pro * segment_radius;
            %             orig
%             lever = -(r_seg-obj.sphere_center)/norm(r_seg-obj.sphere_center);%for now
%             gap = norm(r_seg-obj.sphere_center)-(obj.sphere_radius+segment_radius);
        end

        function v_force_dir_norm = get_force_direction_norm(obj,y_seg,lever)
%             v_force_dir_norm = (obj.sphere_center-y_seg)/norm(obj.sphere_center-y_seg);
            r_seg = y_seg(1:3);
            x_obj_closest = obj.cylinder_point+(r_seg-obj.cylinder_point)'*obj.cylinder_axis*obj.cylinder_axis;
            contact_point = r_seg+lever;
            v_force_dir_norm = (contact_point-x_obj_closest)/norm(contact_point-x_obj_closest);

        end
        function v_force_dir_fric = get_force_direction_fric(obj,y_seg,prev_state,v_force_dir_norm,w_lever)
            %compute direction of tangential contact force (friction).
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
            R_lever = [-cross(R_seg(:,3),w_lever)/norm(cross(R_seg(:,3),w_lever)) w_lever/norm(w_lever) R_seg(:,3)];
           
            %eq. 16:
            R_rot =  R_lever'*R_seg; % Kontaktpunkt im lokalen Koordinatensystem (Transformation matrix (rotation))
            RR_old = (R_rot*R_seg_old')'; % Rotationsmatrix für Kontaktpunkt im vorherigen Schritt im lokalen Koordinatensystem
            
            %eq. 17:
            w_lever_old = segment_radius * RR_old(:,2);
            
            %eq. 14:
            contact_point_old = r_seg_old +  w_lever_old; %point of contact in previous simulation step
            
            % eq. 13: direction of motion of contact point
            w_motion = (contact_point-contact_point_old);

            % eq. 18: project onto object's surface (normalized in paper, but thats wrong)
            v_force_dir_fric = w_motion - w_motion'*v_force_dir_norm * v_force_dir_norm;       
        end
        

    end
end

