function fig = plot_bbc(y,robot,ind)
%% function visualize(s,y,robot,bc,varargin)
%
%   input:
%       -s          material koordinate
%       -y:         states of robot
%       -robot:     obj of class Robot
%       -bc:        obj of class BoundaryCondition
%
%
%   output: figure with nice plot ;)
%
%

% check input
chk_y_dim = size(y) == robot.n_nodes;
assert(any(chk_y_dim),...
    'Mismatch between util.N_nodes and given data for state-space-rep in solution.')

% Flip dimensions if required
if chk_y_dim(1)
    y = y';
end

% fig.Color = 'white';
% 3D Plot
p3d = plot3(y(1,:),y(2,:),y(3,:));
p3d.LineWidth = 2;

%if bool_bc,plot_bc_r(bc),end % boundary conditions
sc = 0:pi/10:2*pi;
hold on;
grid on;
r = robot.segments{2}.segment_radius;
p3d.Color = [1 0.188 0];%[0.85 0.3250 0.098];%'black';
for i = ind:ind:size(y,2)
    hL = y(4:7,i);
    R_i = (quat2rot(hL));
    t = R_i(:,3);
    phi = atan2(t(2),t(1)); %azimuth angle, in [-pi, pi]
    theta = atan2(sqrt(t(1)^2 + t(2)^2) ,t(3));% zenith angle, in [0,pi]
    xunit = y(1,i)- r*( cos(sc)*sin(phi) + sin(sc)*cos(theta)*cos(phi) );
    yunit = y(2,i)+ r*( cos(sc)*cos(phi) - sin(sc)*cos(theta)*sin(phi) );
    zunit = y(3,i)+ r*sin(sc)*sin(theta);
    plot3(xunit, yunit, zunit,'Color',0.5*[1 1 1])
end


ax = gca;
limits = [ax(1).XLim;ax(1).YLim];
ax.XLim = [-max(max(abs(limits))),max(max(abs(limits)))];
ax.YLim = [-max(max(abs(limits))),max(max(abs(limits)))];
ax.ZLim = [0,0.15];
ax.XLabel.String = '$x$ in m'; 
% ax.YLabel.String  = '$y$ in m'; 
ax.ZLabel.String  = '$z$ in m'; 
axis image
view([1 0])
hold on
end

%% Local functions



