function s = compute_s_for_rob(robot,y)
%COMPUTE_S_FOR_ROB Creates vector of nodes for materialpoint s that match
%output of states y of the simulation
s =[];
sL_last_seg = 0;

for i_seg = 1:robot.n_segments
    segment = robot.segments{i_seg};
    stepsize = segment.stepsize;
    s_seg = zeros(segment.n_nodes_seg,1);
    for i_node = 1:segment.n_nodes_seg     
        s_seg(i_node) = sL_last_seg + (i_node - 1)*stepsize;  
    end
    sL_last_seg = sL_last_seg + (segment.n_nodes_seg - 1)*stepsize;
    s = [s;s_seg];
end

[n_nodes, ~] = size(y);
assert(length(s)==n_nodes,'Number of nodes in y doesnt match number of nodes in s!')
end

