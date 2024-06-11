function drawObstacle3D(xObs,yObs,dSafe)
%DRAWOBSTACLE3D Plot the obtacle placed in (xObs, yObs) and the unsafe
%region whihc is defined as a circle with radiud d_{Safe} and center
% (xObs, yObs)
%-------------------------------------------------------------------------
%Input:
% xObs : x-position of the obstacle
% yObs : y-position of the obstacle
% dSafe : safety distance d_{Safe}
%=========================================================================

% Draw a filled circle with specified opacity on the XY plane
theta = linspace(0, 2*pi, 100);
x = xObs + dSafe * cos(theta);
y = yObs + dSafe * sin(theta);
z_ground = zeros(size(theta));
fill3(x, y, z_ground, [0.706, 0.706, 0.722], 'FaceAlpha', 1, 'EdgeColor', 'none');

% Plot the position of the obstacle with a cross
plot(xObs, yObs, 'kx');

end

