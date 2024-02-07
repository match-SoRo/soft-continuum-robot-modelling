function [r0i, h0i, n0i, m0i] = set_initial_values(optim_params, bc)
% set all initial states either from bc if bc contains it
% or from initial_guess in optim params

if bc.has_value('r0')
    r0i = bc.r0;
else
    r0i = optim_params(1:3);
    optim_params = optim_params(4:end);
end
if bc.has_value('h0')
    h0i = bc.h0;
else    
    h0i = optim_params(1:4);
    h0i = h0i/norm(h0i);
    optim_params = optim_params(5:end);
end
if bc.has_value('n0')
    n0i = bc.n0;
else
    n0i = optim_params(1:3);
    optim_params = optim_params(4:end);
end
if bc.has_value('m0')
    m0i = bc.m0;
else
    m0i = optim_params(1:3);
    optim_params = optim_params(4:end);
end

end