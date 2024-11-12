syms l1 l2 l3 l4 l5 q1 q2 q3 q4
%MDH(anpha,a,d,theta)
%T01=MDH(pi/2,0, l1,q1)
T12=MDH(0, 0, 0, q2)
T23=MDH(0, l2,0, q3)
T34=MDH(0, l3,0, -q2 -q3 )
%T45=MDH(0, l4,0, -pi/2)
T5E=MDH(0, l5,0, 0)

T01 =[
 
[cos(q1), -sin(q1), 0,0]
[0, 0, -1, -l1]
[sin(q1),cos(q1), 0, 0]
[ 0, 0, 0,1]]
 

T45 =[
 
[ 0, 1, 0,  l4]
[-1, 0, 0,  0]
[ 0, 0, 1,  0]
[ 0, 0, 0,  1]]



T0E = simplify(T01*T12*T23*T34*T45*T5E)
