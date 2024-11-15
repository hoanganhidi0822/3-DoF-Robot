% q1 = 0;
% q2 = pi/2;
% q3 =-pi/2;
% l1 = 7.5;
% l2 = 20;
% l3 = 22;
% l4 = 4.5;
% l5 = 2.0;
 
T0E =  [[ 0, cos(q1),  sin(q1), cos(q1)*(l4 + l3*cos(q2 + q3) + l2*cos(q2))]
        [ 0, sin(q1), -cos(q1), sin(q1)*(l4 + l3*cos(q2 + q3) + l2*cos(q2))]
        [-1,       0,        0,      l1 - l5 + l3*sin(q2 + q3) + l2*sin(q2)]
        [ 0,       0,        0,                                           1]];
a = simplify(T0E)