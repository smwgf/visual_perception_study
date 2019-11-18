function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly



A = [];
x_c = (K\([x ones(size(x,1),1)]'))';
% DLT
for i = 1:size(x_c,1)
  a_x = [-X(i,:)*x_c(i,2) -x_c(i,2) 0 0 0 0 X(i,:)*x_c(i,1) x_c(i,1)];
  a_y = [0 0 0 0 -X(i,:)*x_c(i,2) -x_c(i,2) X(i,:)*x_c(i,2) x_c(i,2)];
  A = [A;a_x;a_y];
end

[U,S,V] = svd(A);
P = [V(1,end) V(2,end) V(3,end) V(4,end);V(5,end) V(6,end) V(7,end) V(8,end); V(9,end) V(10,end) V(11,end) V(12,end)];
R_= P(:,1:3);
T_=P(:,4);

[U,S,V] = svd(R_);
R =[];
C = [];
if (det(U*V')==1)
  R=U*V';
  C = T_/S(1,1);
else
  R = -U*V';
  C = -T_/S(1,1);
end