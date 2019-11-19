function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations

  X=[];
  for i=1:size(X0,1)
    X=[X;Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:))];
  end
end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
  X=X0;
  pre_error = 0;
  for i=1:1000
    rep_x1 = K*R1*(X'-C1);
    rep_x2 = K*R2*(X'-C2);
    rep_x3 = K*R3*(X'-C3);
    b = [x1(1) x1(2) x2(1) x2(2) x3(1) x3(2)]';
    Fx = [rep_x1(1)/rep_x1(3) rep_x1(2)/rep_x1(3) rep_x2(1)/rep_x2(3) rep_x2(2)/rep_x2(3) rep_x3(1)/rep_x3(3) rep_x3(2)/rep_x3(3)]';
    e = (b-Fx);
    sqr_e =sqrt(e'*e);
    %printf("error : %f \r\n",sqrt(sqr_e));
    if (sqr_e == pre_error)
      break;
    end
    pre_error = sqr_e;
    J=[Jacobian_Triangulation(C1,R1,K,rep_x1)' Jacobian_Triangulation(C2,R2,K,rep_x2)' Jacobian_Triangulation(C3,R3,K,rep_x3)']';
##    if(det(J'*J)<1.0e-10)
##      break;
##    end
    dX = pinv(J'*J)*J'*e;
    X= (X' + dX)';
  end
end

function df = Jacobian_Triangulation(C, R, K, x)
% x is reprojection point in camera coordinate
    f = K(1,1);
    px = K(1,3);
    py = K(2,3);
    u = x(1)/x(3);
    v = x(2)/x(3);
    w = 1;
    
    du = [f*R(1,1)+px*R(3,1) f*R(1,2)+px*R(3,2) f*R(1,3)+px*R(3,3)];
    dv = [f*R(2,1)+py*R(3,1) f*R(2,2)+py*R(3,2) f*R(2,3)+py*R(3,3)];
    dw = [R(3,1) R(3,2) R(3,3)];
    df = [(w*du-u*dw)./(w^2);(w*dv-v*dw)./(w^2)];
end
