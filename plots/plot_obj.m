for i_object = 1:numel(objects)
    if isa(objects{i_object},'envSphere')
        rObs = objects{i_object}.sphere_radius;
        cObs = objects{i_object}.sphere_center;
        [X,Y,Z] = sphere;
        X2 = X * rObs; Y2 = Y * rObs; Z2 = Z * rObs;
        surf(X2+cObs(1),Y2+cObs(2),Z2+cObs(3),'FaceColor',0.5*[1 1 1],'FaceAlpha',visS,'EdgeAlpha',visS);%,'EdgeColor','none')
        hold on
    elseif isa(objects{i_object},'envWall')
        v = objects{i_object}.wall_normal';
        x1 = objects{i_object}.wall_point(1);
        y1 = objects{i_object}.wall_point(2);
        z1 = objects{i_object}.wall_point(3);
        w = null(v); % Find two orthonormal vectors which are orthogonal to v
        zminmax = zlim;
        [Q1,Q2] = meshgrid(-0.02:0.01:0.1,0:0.01:zminmax(2));
        X = x1+w(1,1)*Q1+w(1,2)*Q2; % Compute the corresponding cartesian coordinates
        Y = y1+w(2,1)*Q1+w(2,2)*Q2; %   using the two vectors in w
        Z = z1+w(3,1)*Q1+w(3,2)*Q2;
        surf(X,Y,Z,'FaceColor',0.5*[1 1 1],'FaceAlpha',visW,'EdgeAlpha',visW)
        hold on
    elseif isa(objects{i_object},'envCylinder')
        rObs = objects{i_object}.cylinder_radius;
        cObs = objects{i_object}.cylinder_point;
        aObs = objects{i_object}.cylinder_axis;
        [X,Y,Z] = cylinder(rObs);
        Z = Z*0.05-0.025;
        x_ = [0; 0; 0];
        xObs_ = cObs+(-cObs)'*aObs*aObs;
        vxObs = xObs_/norm(xObs_);
        Rcyl = [vxObs cross(vxObs,aObs) aObs];
        XR1 = Rcyl*[X(1,:);Y(1,:);Z(1,:)];
        XR2 = Rcyl*[X(2,:);Y(2,:);Z(2,:)];
        X2 = [XR1(1,:); XR2(1,:)];
        Y2 = [XR1(2,:); XR2(2,:)];
        Z2 = [XR1(3,:); XR2(3,:)];
        X3 = X2+cObs(1); Y3 = Y2+cObs(2); Z3 = Z2+cObs(3);
        surf(X3,Y3,Z3,'FaceColor',0.5*[1 1 1],'FaceAlpha',visC,'EdgeAlpha',visC);%,'EdgeColor','none')
        fill3(X3(1,:),Y3(1,:),Z3(1,:),0.5*[1 1 1],'FaceAlpha',visC,'EdgeAlpha',visC)
        fill3(X3(2,:),Y3(2,:),Z3(2,:),0.5*[1 1 1],'FaceAlpha',visC,'EdgeAlpha',visC)
        hold on
    end
end