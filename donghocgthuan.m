%%programmed by Nguyen Hoang Minh Triet
%%khai bao ma tran tong quat
%function T = standardDH(d,theta,a, anpha)
%T=[cos(theta),-cos(anpha)*sin(theta),sin(anpha)*sin(theta),a*cos(theta);...
%sin(theta),cos(anpha)*cos(theta),-sin(anpha)*cos(theta),a*sin(theta);...
%0, sin(anpha), cos(anpha),d;...
%0,0,0,1];
%end

%%khai bao bien va cac ma tran bien doi thuan nhat
syms C1 S1 C2 S2 C3 S3 L1 L2 L3 L4 L5 C4 S4;

T01 = [C1 -S1 0 0; 
       S1  C1 0 0; 
       0   0  1 L1; 
       0   0  0 1];

T12 = [C2 -S2 0 0;
       0   0 -1 0; 
       S2  C2 0 0; 
       0   0  0 1];

T23 = [C3 -S3 0 L2; 
       S3  C3 0 0; 
       0   0  1 0; 
       0   0  0 1];

T34 = [C4 -S4 0 L3; 
       S4  C4 0 0; 
       0   0  1 0; 
       0   0  0 1];

T45 = [0 1 0 L4; 
      -1 0 0 0; 
       0 0 1 0; 
       0 0 0 1];

T5EE = [1 0 0 L5; 
        0 1 0 0; 
        0 0 1 0; 
        0 0 0 1];

%%tinh toan ma tran chuyen vi
T0EE = T01*T12*T23*T34*T45*T5EE;
simplify(simplify(T0EE))

%%ket qua
%[C4*(C1*C2*S3 + C1*C3*S2) + S4*(C1*C2*C3 - C1*S2*S3), C4*(C1*C2*C3 - C1*S2*S3) - S4*(C1*C2*S3 + C1*C3*S2),  S1, L4*(C4*(C1*C2*C3 - C1*S2*S3) - S4*(C1*C2*S3 + C1*C3*S2)) + L5*(C4*(C1*C2*S3 + C1*C3*S2) + S4*(C1*C2*C3 - C1*S2*S3)) + L3*(C1*C2*C3 - C1*S2*S3) + C1*C2*L2]
%[C4*(C2*S1*S3 + C3*S1*S2) + S4*(C2*C3*S1 - S1*S2*S3), C4*(C2*C3*S1 - S1*S2*S3) - S4*(C2*S1*S3 + C3*S1*S2), -C1, L4*(C4*(C2*C3*S1 - S1*S2*S3) - S4*(C2*S1*S3 + C3*S1*S2)) + L5*(C4*(C2*S1*S3 + C3*S1*S2) + S4*(C2*C3*S1 - S1*S2*S3)) + L3*(C2*C3*S1 - S1*S2*S3) + C2*L2*S1]
%[            C4*(S2*S3 - C2*C3) + S4*(C2*S3 + C3*S2),             C4*(C2*S3 + C3*S2) - S4*(S2*S3 - C2*C3),   0,                             L1 + L4*(C4*(C2*S3 + C3*S2) - S4*(S2*S3 - C2*C3)) + L5*(C4*(S2*S3 - C2*C3) + S4*(C2*S3 + C3*S2)) + L3*(C2*S3 + C3*S2) + L2*S2]
%[                                                  0,                                                   0,   0,                                                                                                                                                         1]