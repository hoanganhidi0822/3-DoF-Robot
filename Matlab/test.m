syms l0x l0y l1 l2 l3 l4 l5 q1 q2 q3 q4 q5
%MDH(anpha,a,d,theta)
T01=MDH(0,     0,   l1, q1)
T12=MDH(   sym(pi/2),     0,    0, q2);
T23=MDH(   0,    l2,    0, q3)
T34=MDH(   0,    l3,    0, -q2 -q3 )
T45=MDH(   0,    l4,    0, -sym(pi/2))
T5E=MDH(   0,    l5,    0, 0)

T01 =[
 
[cos(q1), -sin(q1),  0,   0]
[sin(q1),  cos(q1),  0,   0]
[  0,        0,      1,   l1]
[ 0     ,    0,      0,   1]]

T12 = [ 
[cos(q2), -sin(q2), 0, 0]
[  0,        0,    -1, 0]
[sin(q2),  cos(q2), 0, 0]
[0,          0,     0, 1]]

T45 =[[ 0, 1, 0,  l4]
      [-1, 0, 0,  0 ]
      [ 0, 0, 1,  0 ]
      [ 0, 0, 0,  1 ]]
q5 = -1;
WT0 = [cosd(q5) 0 sind(q5) l0x
       0        1  0       l0y
      -sind(q5) 0 cosd(q5)  0 
       0        0  0        0 ]

T0E = WT0 * simplify(T01*T12*T23*T34*T45*T5E)
