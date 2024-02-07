classdef Load
    % LOAD describes one pressure sample for a static configuration
    %  it contains the actuation pressure as well as all external forces
    %  moments and additional masses on the Robot
    
        
    
    properties
        pressure % given pressure for all three chambers (dim: 3xn_act_segments)
        n_ext % external forces (3 x nr of s_n_ext)
        s_n_ext % list of points s on the robot where the force is applied
        m_ext % external moments (3 x nr of s_m_ext)
        s_m_ext % list of points s on the robot where the moment is applied
        tip_mass % additions mass at the tip (3x1)
        gravity % (3x1)
    end
    
    methods
        function obj = Load(pressure, gravity, tip_mass, n_ext, m_ext, s_n_ext, s_m_ext)
            
            if size(n_ext,2) ~= max(size(s_n_ext)) || size(m_ext,2) ~= max(size(s_m_ext))
                error('number of external forces and and points on the actuator of external forces do not match');
            end
            obj.pressure = pressure;
            if isempty(gravity)
                obj.gravity = [0;0;0];
            else
                obj.gravity = gravity;
            end
            obj.tip_mass = tip_mass;
            obj.n_ext = n_ext;
            obj.m_ext= m_ext;
            obj.s_n_ext = s_n_ext;
            obj.s_m_ext = s_m_ext;

            
        end
        

    end
end

