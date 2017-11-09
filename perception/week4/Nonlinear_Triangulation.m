function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

F = [];
N=size(x1,1);
%X=zeros(N,3);
for i = 1:N
    X   = X0(i,:)';
    X1 = x1(i,:)';
    X2 = x2(i,:)';
    X3 = x3(i,:)';
    W = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, X1, X2, X3, X);
    F = [F;W'];
end
X = F;
    
    


end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
X=X0;
for i =1:10
    J1 = Jacobian_Triangulation(C1, R1, K, X);
    J2 = Jacobian_Triangulation(C2, R2, K, X);
    J3 = Jacobian_Triangulation(C3, R3, K, X);
    J = [J1;J2;J3];
    b = [x1(1) x1(2) x2(1) x2(2) x3(1) x3(2)]';
    UVW1 = K*R1*(X-C1); u1=UVW1(1); v1=UVW1(2); w1=UVW1(3);
    UVW2 = K*R2*(X-C2); u2=UVW2(1); v2=UVW2(2); w2=UVW2(3);
    UVW3 = K*R3*(X-C3); u3=UVW3(1); v3=UVW3(2); w3=UVW3(3);
    fX = [u1/w1 v1/w1 u2/w2 v2/w2 u3/w3 v3/w3]';
    DX = (J'*J)\(J')*(b-fX);
    X = X + DX;
end

end

function Jivec = Jacobian_Triangulation(C, R, K, X)

UVW = K*R*(X-C); u = UVW(1); v=UVW(2); w=UVW(3);
f = K(1,1); px=K(1,3); py=K(2,3);
r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);

thu = [f*r11+px*r31 f*r12+px*r32 f*r13+px*r33];
thv  = [f*r21+py*r31 f*r22+py*r32 f*r23+py*r33];
thw = [r31 r32 r33];

Jivec = [(w*thu - u*thw)/w^2; (w*thv - v*thw)/w^2];


end












