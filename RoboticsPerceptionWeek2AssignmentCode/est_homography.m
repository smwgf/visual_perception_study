function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
T=zeros(8,9);
for i = 1 : 4
  Xi=video_pts(i,1);
  Yi=video_pts(i,2);
  x_i=logo_pts(i,1);
  y_i=logo_pts(i,2);
  T((i-1)*2+1,:)=[-Xi -Yi -1 0 0 0 Xi*x_i Yi*x_i x_i];
  T((i-1)*2+2,:)=[0 0 0 -Xi -Yi -1 Xi*y_i Yi*y_i y_i];
end

[U, S, V] = svd(T);
H = [V(1,9) V(2,9) V(3,9);V(4,9) V(5,9) V(6,9);V(7,9) V(8,9) V(9,9)];
end

