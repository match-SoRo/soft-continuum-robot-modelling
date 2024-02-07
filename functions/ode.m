function ys = ode(y, robot, robot_load, i_seg, s_vec, i_node_seg, par_se, par_bt)
    % y: state of the robot at s+ds
    % i_seg: index of the currently evaluated segment
    % s_vec: vector containing positions of nodes along whole s
    % i_node_seg: index of current node
    % par_se: shear extension
    % par_bt: bending torsion

    ys = zeros(robot.n_states,1);
    
    h = y(4:7);
    h = h/norm(h);
    n = y(8:10);
    m = y(11:13);
    R = quat2rot(h);
                
    v_star = robot.segments{i_seg}.v_star;
    u_star = robot.segments{i_seg}.u_star;
    v = (1./par_se).*(R'*n)+v_star;
    u = (1./par_bt).*(R'*m)+u_star;
    
    % kinematic constraints
    % ...
        
    fg = robot.segments{i_seg}.material.rhoA*robot_load.gravity;
            
    %ODE System
    rs = R*v;
    Rs = R*hat(u);
    fp = [0;0;0];
    f_ext = [0;0;0];
    lp = [0;0;0];
    l_ext = [0;0;0];
    
   if isa(robot.segments{i_seg}, 'Actuator')
        for i_chamb=1:3
            fp_axial = -robot_load.pressure(i_chamb,i_seg)*robot.segments{i_seg}.chamber_area * robot.segments{i_seg}.axial_factor;
            fp_radial = -robot.segments{i_seg}.shear_factor*robot_load.pressure(i_chamb,i_seg);

            fp = fp + ((fp_axial+fp_radial)*Rs*[0; 0; 1]);
            lp = lp + ((fp_axial+fp_radial)*R*(cross(v+cross(u,robot.segments{i_seg}.chamber_pos(:,i_chamb)),[0;0;1])+cross(robot.segments{i_seg}.chamber_pos(:,i_chamb),cross(u,[0;0;1]))));
        end
    end

    ds = robot.segments{i_seg}.stepsize;
    s = s_vec(i_node_seg);
    s_ds = s_vec(i_node_seg+1);
    
    for i_n_ext = 1:size(robot_load.n_ext,2)
        if ds > 0 && s_ds > robot_load.s_n_ext(i_n_ext) && s <= robot_load.s_n_ext(i_n_ext)
            f_ext = robot_load.n_ext(:,i_n_ext)/ds;
        elseif ds < 0 && s_ds <= robot_load.s_n_ext(i_n_ext) && s > robot_load.s_n_ext(i_n_ext)
            f_ext = -robot_load.n_ext(:,i_n_ext)/ds;
        end
    end
    
    for i_m_ext = 1:size(robot_load.m_ext,2)
        if ds > 0 && s_ds > robot_load.s_m_ext(i_m_ext) && s <= robot_load.s_m_ext(i_m_ext)
            l_ext = robot_load.m_ext(:,i_m_ext)/ds;
        elseif ds < 0 && s_ds <= robot_load.s_m_ext(i_m_ext) && s > robot_load.s_m_ext(i_m_ext)
            l_ext = -robot_load.m_ext(:,i_m_ext)/ds;
        end
    end
    
    
    ns = - fg - fp - f_ext;
    ms = - cross(rs,n) - lp - l_ext;
            
    % R^3 to Quaternion
    hs = [ 0, -u(1), -u(2), -u(3);
        u(1), 0, u(3), -u(2);
        u(2), -u(3), 0, u(1);
        u(3), u(2), -u(1), 0 ] * h/2;

    ys = [rs; hs; ns; ms];
            
end

