% A = [1 0 0;0 1 0; 1 1 1];
% B = [1;1;1];

T = [-2 0 1;0 1 -1; -1 2 -1];
A=T'
B = [-1;1;1];

result = (A'*A)\(A'*B);

C=A;
for i = 1 : 3
    C(:,i)=A(:,i)*result(i);
end;

%% find rotation matrix

A = [1 1 -1 -1; 1 -1 -1 1; 1 1 1 1];
B = [-1.2131 -1.4413 0.3470 0.5752; 0.0851 -0.7858 -1.6594 -0.7885; -1.2334 0.5525 0.3550 -1.4309];

[U,S,V] = svd(B*A');

R =V*U';

%% compute essential matrix
T = [1 1 0];
get_skewed_matrix(T)*[ cos(0.5) -sin(0.5) 0; sin(0.5) cos(0.5) 0; 0 0 1];

T=[-1 2 0];
get_skewed_matrix(T)

