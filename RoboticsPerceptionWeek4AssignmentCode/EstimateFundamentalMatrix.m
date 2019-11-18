function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2


A = [x1(:,1).*x2(:,1) x1(:,1).*x2(:,2) x1(:,1) x1(:,2).*x2(:,1) x1(:,2).*x2(:,2) x1(:,2) x2(:,1) x2(:,2) ones(size(x1,1),1)];
[U,S,V] = svd(A);
F_ = [V(1,9) V(2,9) V(3,9);V(4,9) V(5,9) V(6,9);V(7,9) V(8,9) V(9,9)];

[U,S,V] = svd(F_);
S(3,3) = 0;
F = U*S*V';
F = F./norm(F);