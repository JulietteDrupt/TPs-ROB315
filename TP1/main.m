d3 = 0.7;
r1 = 0.5;
r4 = 0.2;

alpha = [ 0 pi/2 0 pi/2 -pi/2 pi/2 ];
d = [ 0 0 d3 0 0 0 ];
theta = [ -pi/2 0 pi/2-pi/2 -pi/2 -pi/2 -pi/2 ];
r = [ r1 0 0 r4 0 0 ];

g_06 = CalculMGD(alpha,d,theta,r);

%Q3
r_E = 0.1;
g_6E = CalculTransformationElem(0,0,0,r_E);
g_0E = g_06*g_6E;
disp(g_0E);

%Q4
P_0E = g_0E(1:3, 4);
R_0E = g_0E(1:3, 1:3);
q = atan2(0.5 * sqrt((R_0E(3,2) - R_0E(2,3))^2 + (R_0E(1,3) - R_0E(3,1))^2 + (R_0E(2,1) - R_0E(1,2))^2), 0.5 * (R_0E(1,1) + R_0E(2,2) + R_0E(3,3) - 1));
n = [ (R_0E(3,2)-R_0E(2,3))/(2*sin(q)) (R_0E(1,3)-R_0E(3,1))/(2*sin(q)) (R_0E(2,1)-R_0E(1,2))/(2*sin(q)) ];
disp(q);
disp(n);
