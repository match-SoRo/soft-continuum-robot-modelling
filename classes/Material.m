classdef Material
    %MATERIAL describes material of a segment
    %   material properties and material law
    
    properties(Access = public)
        material_name %ecoflex/dragon skin
        mat_law_type % 1:lin, 2:ANN, 3:nonlin
        rho  % material constant 
        rhoA % rho*A
        E % modulus of elasticity
        G % shear modulus
        K_se % shear extension stiffnes matrix
        K_bt % bending torsion stiffnes matrix
        K_se_inv
        K_bt_inv
        v0
        u0
        
        
    end
    
    methods(Access = public)
        
        function obj = Material(material_name, mat_law_type, E, G, rho)
            % Constructor
            obj.material_name = material_name;
            obj.mat_law_type = mat_law_type;
            obj.G = G;
            obj.E = E;
            obj.rho = rho;
                     
            

        end
        
        

    end
    
    
    methods(Access = {?Segment})
        
        function obj = calc_K_se_K_bt(obj, segment_area, inertia)
            
            obj.K_se = diag([obj.G*segment_area, obj.G*segment_area, obj.E*segment_area]);      
            obj.K_bt = diag([obj.E*inertia(1,1), obj.E*inertia(2,2), obj.G*inertia(3,3)]);
            obj.K_se_inv = inv(obj.K_se);
            obj.K_bt_inv = inv(obj.K_bt);
            obj.rhoA = obj.rho * segment_area;
            
        end
        
    end
        

    
    
    
end

