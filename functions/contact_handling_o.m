function [f_temp_n, f_temp_t, y_contact, x_contact, gap_out] = contact_handling(objects, y_seg, segment_radius, prev_state, b_isTip)

%(objects, y_seg, i_node_seg, i_seg, n_nodes_seg, n_segments, segment_radius, prev_state)
%check for individual node
y_tip       = zeros(6,1);
x_contact   = zeros(3,1);
f_contact   = zeros(3,1); %contact force global coord sys
m_contact   = zeros(3,1); %contact moment global coord sys
y_contact   = [f_contact; m_contact];
f_temp_n    = zeros(3,1); %force normal contact global coord sys
f_temp_t    = zeros(3,1); %force tangential contact global coord sys
gap_out     = 0;

%check for contact with objects
for i_object = 1:numel(objects)
    k_penalty       = objects{i_object}.penalty_stiffness;
    thresh          = objects{i_object}.thresh;
    p_reg           = objects{i_object}.p_reg;                          %regularization parameter penalty force
    fric_coeff      = objects{i_object}.get_fric();                     %friction coefficient
    [gap, lever]    = objects{i_object}.get_gap(y_seg,segment_radius);  %Basiert auf disk um node. Scheibe (Flacher Zylinder)
    if gap < 0
        gap_out=gap;
        
        v_force_dir_norm = objects{i_object}.get_force_direction_norm(y_seg,lever);
        [v_force_dir_fric] = objects{i_object}.get_force_direction_fric(y_seg,prev_state,-v_force_dir_norm,lever); %Reibungskontakt!
        x_contact = y_seg(1:3)+lever;
        
        % Normal penalty force (with regularization)
        if gap >= -p_reg 
            f_temp_n    =  k_penalty/(2*p_reg)*(gap^2)*v_force_dir_norm;
        else
            f_temp_n    = -k_penalty*(gap+p_reg/2) * v_force_dir_norm;
        end

        mu = step(v_force_dir_fric,-thresh,-fric_coeff,thresh,fric_coeff);
        f_temp_t = -norm(f_temp_n)*mu*v_force_dir_fric/norm(v_force_dir_fric);

    
        f_temp          = f_temp_n + f_temp_t;   % total force global coord sys
        m_temp          = cross(lever,f_temp);   % total moment global coord sys

        f_contact       = f_contact + f_temp;
        m_contact       = m_contact + m_temp;

        y_contact       = [f_contact; m_contact];    
        if b_isTip ~= 0
            y_tip(1:3) = f_contact; 
            y_tip(4:6) = m_contact;
        end

    end

end
end


function h = step(x,x0,h0,x1,h1)
if norm(x)<=x0
    h = h0;
elseif norm(x)<x1 && norm(x)>x0
    x2 = (norm(x)-x0)/(x1-x0);
    h = h0 + (3*x2^2-2*x2^3)*(h1-h0);
else	
    h = h1;
end
end
