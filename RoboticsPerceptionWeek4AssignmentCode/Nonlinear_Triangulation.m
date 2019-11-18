function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
end

function J = Jacobian_Triangulation(C, R, K, x)
% x is reprojection point in camera coordinate
    f = K(1,0);
    px = K(1,3);
    py = K(2,3);
    u = x(1);
    v = x(2);
    w = x(3);
    
    du = [f*R(1,1)+px*R(3,1) f*R(1,2)+px*R(3,2) f*R(1,3)+px*R(3,3)];
    dv = [f*R(2,1)+py*R(3,1) f*R(2,2)+py*R(3,2) f*R(2,3)+py*R(3,3)];
    dw = [R(3,1) R(3,2) R(3,3)];
    J = [(w*du-u*dw)./w^2;(w*dv-v*dw)./w^2]';
end
