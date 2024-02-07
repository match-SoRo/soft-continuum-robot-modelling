function [residual, y, contactStruct] = get_residual_sim(optim_params, robot, objects, i_sample, bc, prev_state)
% get_residual_sim is the objective function of the shooting method for
% the simulation which optimizes initial condition
% optim_params contains initial values n0 (force) and m0 (moment)
y_contact = zeros(6,1);
y = zeros(robot.n_states, robot.n_nodes);
[r0i, h0i, n0i, m0i] = set_initial_values(optim_params, bc);
y_tip = zeros(6,1);

b_isTip = 0;
contactStruct.gap = zeros(1, robot.n_nodes);
contactStruct.X = zeros(3, robot.n_nodes);
contactStruct.F_n = zeros(3, robot.n_nodes);
contactStruct.F_t = zeros(3, robot.n_nodes);
contactStruct.m_contact = zeros(3, robot.n_nodes);

robot_load = robot.robot_loads{i_sample};

i_node = 1;
for i_seg = 1:robot.n_segments
    i_next_seg = i_seg+1;

    segment = robot.segments{i_seg};
    
    y_seg = zeros(robot.n_states, segment.n_nodes_seg);
    y_seg(:,1) = [r0i; h0i; n0i; m0i];
    
    stepsize = segment.stepsize;
    s_vec = segment.s_vec;
    
    for i_node_seg = 1:segment.n_nodes_seg-1
         
        % Runge Kutta
        yj = y_seg(:,i_node_seg);
        ode_h = @ode;
        
        [par_se, par_bt] = call_material_fcn(segment, yj, s_vec(i_node_seg), robot_load.pressure(:,i_seg), segment.material.mat_law_type);
        k1 = ode_h(yj, robot, robot_load, i_seg, s_vec, i_node_seg, par_se, par_bt);
        
        [par_se, par_bt] = call_material_fcn(segment, yj+k1*stepsize/2, s_vec(i_node_seg)+stepsize/2, robot_load.pressure(:,i_seg), segment.material.mat_law_type);
        k2 = ode_h(yj+k1*stepsize/2, robot, robot_load, i_seg, s_vec, i_node_seg, par_se, par_bt);
        
        [par_se, par_bt] = call_material_fcn(segment, yj+k2*stepsize/2, s_vec(i_node_seg)+stepsize/2, robot_load.pressure(:,i_seg), segment.material.mat_law_type);
        k3 = ode_h(yj+k2*stepsize/2, robot, robot_load, i_seg, s_vec, i_node_seg, par_se, par_bt);
        
        [par_se, par_bt] = call_material_fcn(segment, yj+k3*stepsize, s_vec(i_node_seg)+stepsize, robot_load.pressure(:,i_seg), segment.material.mat_law_type);
        k4 = ode_h(yj+k3*stepsize, robot, robot_load, i_seg, s_vec, i_node_seg, par_se, par_bt);
        
        y_seg(:,i_node_seg+1) = yj + (stepsize*(k1 + 2*(k2+k3) + k4))/6;
        
        %Kontakt
        [f_temp_n,f_temp_t,y_contact,x_contact,gap] = contact_handling(objects, y_seg(:,i_node_seg+1), segment.segment_radius, prev_state(:,i_node-1+i_node_seg+1), b_isTip);%(objects, y_seg(:,i_node_seg+1), i_node_seg, i_seg, segment.n_nodes_seg, robot.n_segments, segment.segment_radius, prev_state(:,i_node-1+i_node_seg+1));
        if any(y_contact)
            contactStruct.gap(i_node+i_node_seg) = gap;
            contactStruct.X(:,i_node+i_node_seg) = x_contact;
            contactStruct.F_n(:,i_node+i_node_seg) = f_temp_n;
            contactStruct.F_t(:,i_node+i_node_seg) = f_temp_t;
            contactStruct.m_contact(:,i_node+i_node_seg) = y_contact(4:6);
            robot_load.n_ext=y_contact(1:3); 
            robot_load.m_ext=y_contact(4:6);
            robot_load.s_n_ext=s_vec(i_node_seg+1);
            robot_load.s_m_ext=s_vec(i_node_seg+1);
        end
    end
      
    % use last values as initial values for next segment
    r0i = y_seg(1:3, end);
    h0i = y_seg(4:7, end);
    n0i = y_seg(8:10, end);
    m0i = y_seg(11:13, end);
    R0i = quat2rot(h0i);
    

    for i_chamb=1:3
        if isa(segment, 'Actuator')
            % if segment.is_actuated
            fp_axial = robot_load.pressure(i_chamb,i_seg)*segment.chamber_area * segment.axial_factor;
            fp_radial = segment.shear_factor*robot_load.pressure(i_chamb,i_seg);

            n0i = n0i - R0i*((fp_axial+fp_radial)*[0; 0; 1]);
            m0i = m0i - R0i*(hat(segment.chamber_pos(:,i_chamb))*((fp_axial+fp_radial))*[0; 0; 1]);
        end
        if i_next_seg <= robot.n_segments && i_next_seg >= 1 && isa(robot.segments{i_next_seg}, 'Actuator')
            fp_axial = robot_load.pressure(i_chamb,i_next_seg)*robot.segments{i_next_seg}.chamber_area * robot.segments{i_next_seg}.axial_factor;
            fp_radial = robot.segments{i_next_seg}.shear_factor*robot_load.pressure(i_chamb,i_next_seg);

            n0i = n0i + R0i*((fp_axial+fp_radial)*[0; 0; 1]);
            m0i = m0i + R0i*(hat(robot.segments{i_next_seg}.chamber_pos(:,i_chamb))*((fp_axial+fp_radial))*[0; 0; 1]);
        end

    end
    
    y(:, i_node:i_node+segment.n_nodes_seg-1) = y_seg;
    i_node = i_node+segment.n_nodes_seg;
    
end

% calculate residual
residual = [];
if bc.has_value('rL')
    rL = y(1:3,end);
    residual = [residual; bc.rL - rL];
end
if bc.has_value('hL')
    hL = y(4:7,end);
    RL  = quat2rot(hL);
    RL_bc = quat2rot(bc.hL);
    dR = logm(RL'*RL_bc);
    dR = real(dR); % has to be real for codegen
    residual = [residual; dR(3,2);dR(1,3);dR(2,1)];
end
if bc.has_value('nL')
    nL = y(8:10,end);
    bc.nL = bc.nL+y_contact(1:3);
    residual = [residual; bc.nL - nL];
end
if bc.has_value('mL')
    mL = y(11:13,end);
    bc.mL = bc.mL+y_contact(4:6);
    residual = [residual; bc.mL - mL];
end

if numel(residual) == 12
    %"high" displacement and "force"
    empiric_high_disp  = 0.001;  % in m
    empiric_high_force = 0.1;     % in N %0.1
    
    K_virt_trans = empiric_high_force/empiric_high_disp;
    % "high" displacement and "force"
    empiric_high_rot = 30;  % in degrees
    empiric_high_moment  = 0.0100;     % in N
    
    K_virt_rot = empiric_high_moment/(empiric_high_rot/180*pi);
    K_mat_virt = diag([K_virt_trans*ones(1,3),...
        K_virt_rot*ones(1,3),...
        1/K_virt_trans*ones(1,3),...
        1/K_virt_rot*ones(1,3)]);
    residual = K_mat_virt*(residual.^2);
end

end
























































































