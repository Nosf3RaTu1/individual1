% Task 1 individual
clc;clear;
syms a1 a2 a3 d1 d2 d3 theta1 theta2 theta3
% DH table
% initiate by assign value first, given theta 1 = 0, alpha 1 = 90 degree 
theta1 = deg2rad(0);
alpha1 = deg2rad(90);
 
% Convert degree to radians
%------Important: Change the value with different robot configuration-----
A = [a1 d1 alpha1 theta1
     a2 0  0      theta2
     a3 0  0      theta3];
 

% Homogenous transfer functions 
A1 = [cos(A(1,4))    -sin(A(1,4))*cos(A(1,3))    sin(A(1,4))*sin(A(1,3))   A(1,1)*cos(A(1,4))
      sin(A(1,4))     cos(A(1,4))*cos(A(1,3))    -cos(A(1,4))*sin(A(1,3))  A(1,1)*sin(A(1,4))
      0               sin(A(1,3))                 cos(A(1,3))              A(1,2)
      0               0                           0                        1                ];

A2 = [cos(A(2,4))    -sin(A(2,4))*cos(A(2,3))    sin(A(2,4))*sin(A(2,3))   A(2,1)*cos(A(2,4))
      sin(A(2,4))     cos(A(2,4))*cos(A(2,3))    -cos(A(2,4))*sin(A(2,3))  A(2,1)*sin(A(2,4))
      0               sin(A(2,3))                 cos(A(2,3))              A(2,2)
      0               0                           0                        1                ];

A3 = [cos(A(3,4))    -sin(A(3,4))*cos(A(3,3))    sin(A(3,4))*sin(A(3,3))   A(3,1)*cos(A(3,4))
      sin(A(3,4))     cos(A(3,4))*cos(A(3,3))    -cos(A(3,4))*sin(A(3,3))  A(3,1)*sin(A(3,4))
      0               sin(A(3,3))                 cos(A(3,3))              A(3,2)
      0               0                           0                        1                ];
% Matrix multiplication to find matrix transform of end effector
T01 = A1
T02 = A1*A2;
T03 = A1*A2*A3; %Matrix size (4x4)
T02 = simplify(T02)
T03 = simplify(T03)
% Find Jacobian matrix for end Effector (6x3) for each component
Oz0 = [0;0;1];
Oz1 = T01(1:3,3);
Oz2 = T02(1:3,3);
% Position of each component
O0 = [0;0;0];  % Base coordinate frame also using to fulfill the last 3 line of column 1 to fit with matrix dimension otherwise will contract errors
O1 = T01(1:3,4);
O2 = T02(1:3,4);
O3 = T03(1:3,4);
% Create Jacobi matrix
Jab = [Oz0  cross(Oz1,(O3-O1))  cross(Oz2,(O3-O2))
       O0    Oz1                 Oz2];
Jab = simplify(Jab)
% Analytical Derivation for inverse Jacobian matrix
% Since there is only displacement wrt X,Z base coordinate frame. Also
% rotation wrt y axis of base coordinate (2-D planar robot arm).So the
% Jacobian matrix can be reduce to 3x3 matrix with each line will be
% [Vx,Vz,Omega Y].
Jab_reduced = [Jab(1,:)
               Jab(3,:)
               Jab(5,:)];
Jab_reduced = simplify(Jab_reduced)
% Inverse Kinematic can be carry with only square matrix (this case is 3x3
% matrix)
Jab_inverse = simplify(inv(Jab_reduced))




