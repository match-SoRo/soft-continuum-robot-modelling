function [y_contact, f_temp_n, f_temp_t, x_contact, gap_out] = contact_handling(objects, y_seg, segment_radius, prev_state)
%"Describing and Analyzing Mechanical Contact for Continuum Robots Using a Shooting-Based Cosserat Rod Implementation"

%inputs: 
% objects: envObject array,
% y_seg: state of node 13x1 (r,h,n,m)
% segment_radius (radius of soft actuator)
% prev_state: state of node in previous simulation step 13x1 (r,h,n,m)

%outputs: 
% y_contact: 6x1 contact force and moment needed for further simulation. 
% Rest: for visualization/analysis

%Initialization
f_contact   = zeros(3,1); %contact force global coord sys
m_contact   = zeros(3,1); %contact moment global coord sys
y_contact   = [f_contact; m_contact];
x_contact   = zeros(3,1); %point of contact global coord sys

f_temp_n    = zeros(3,1); %force normal contact global coord sys
f_temp_t    = zeros(3,1); %force tangential contact global coord sys
gap_out     = 0;

%check for contact with ALL objects
for i_object = 1:numel(objects)
    k_penalty       = objects{i_object}.penalty_stiffness;              %penalty stiffness (from eq.8)
    p_regn          = objects{i_object}.p_regn;                         %regularization parameter penalty force (from eq.8) 
    p_regt          = objects{i_object}.p_regt;                         %regularization parameter friction (threshhold) (from eq.9)
    fric_coeff      = objects{i_object}.get_fric();                     %friction coefficient (from eq.9)
    [gap, w_lever]    = objects{i_object}.get_gap(y_seg,segment_radius);%Based on disk around node. Depends on geometry of objects 
    if gap < 0 %if there is an overlap
        gap_out = gap;
        
        %w_n:
        v_force_dir_norm = objects{i_object}.get_force_direction_norm(y_seg,w_lever); %direction of normal contact force. Depends on geometry of objects (w_n) (from eq.8)  
        %w_t: direction and distance point in contact traveled in
        %tangential direction to object's surface. Taken as equivalent velocity. 
        v_force_dir_fric = objects{i_object}.get_force_direction_fric(y_seg,prev_state,-v_force_dir_norm,w_lever); %direction of tangential contact force (friction). (from eq.9)
        
        
        % eq.8: Normal penalty force (with regularization) 
        if gap >= -p_regn 
            f_temp_n    =  k_penalty/(2*p_regn)*(gap^2)*v_force_dir_norm;
        else
            f_temp_n    = -k_penalty*(gap+p_regn/2) * v_force_dir_norm;
        end

        %A regularized approach for frictional impact dynamics of flexible multi-link manipulator arms considering the dynamic stiffening effect
        %eq.9: tangential force (with regularization, polynomial step) if equivalent velocity (traveled distance) is smaller
        %than a threshhold the force is regularized down for numerical reasons (no
        %sudden change of force direction) 
        mu = step(v_force_dir_fric,-p_regt,-fric_coeff,p_regt,fric_coeff);
        f_temp_t = -norm(f_temp_n)*mu*v_force_dir_fric/norm(v_force_dir_fric);

        %eq.7:
        f_temp          = f_temp_n + f_temp_t;   % total force on node global coord sys
        %eq.10:
        m_temp          = cross(w_lever,f_temp);   % total moment node global coord sys

        %sum up in case there is contact with multiple objects
        f_contact       = f_contact + f_temp;
        m_contact       = m_contact + m_temp;

        y_contact       = [f_contact; m_contact]; 
        
        % Not important: point of contact only used for later evaluation 
        x_contact = y_seg(1:3)+w_lever;
    end

end
end


function h = step(x,x0,h0,x1,h1)
%polynomial step
if norm(x)<=x0
    h = h0;
elseif norm(x)<x1 && norm(x)>x0
    x2 = (norm(x)-x0)/(x1-x0);
    h = h0 + (3*x2^2-2*x2^3)*(h1-h0);
else	
    h = h1;
end
end
