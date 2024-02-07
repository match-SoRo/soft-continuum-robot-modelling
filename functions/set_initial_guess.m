function initial_guess = set_initial_guess(bc, robot)
%SET_INITIAL_GUESS creates an initial guess for initial values of the
%integration
% set initial values if not given in boundary conditions
initial_guess = [];
if ~bc.has_value('r0')
    initial_guess = [initial_guess;0;0;0];
end
if ~bc.has_value('h0')
    initial_guess = [initial_guess;1;0;0;0];
end
if ~bc.has_value('n0')
    initial_guess = [initial_guess;0;0;0];
end
if ~bc.has_value('m0')
    initial_guess = [initial_guess;0;0;0];
end
end




