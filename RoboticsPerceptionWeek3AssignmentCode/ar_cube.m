function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography

% mason : 
##H1 = H(:,1);
##H2 = H(:,2);
##H3 = cross(H1,H2);
##Rx = [H1,H2,H3];
##[U,X,V] = svd(Rx);
##d = det(U*V');
##R = U*[1 0 0; 0 1 0; 0 0 det]*V';
##t = H(:,3)/.norm(H1);

% YOUR CODE HERE: Project the points using the pose

% mason : proj_points = K * H * render_points

end
