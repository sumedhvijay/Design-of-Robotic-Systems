function findJ = jacobian_Matrix()
% Create symbols for theta.
syms t1 t2 t3 t4 t5; 
pi = sym('pi');

% D-H parameters.
l_1 = 0;
l_2 = 5;
l_3 = 10;
l_4 = 10;
l_5 = 10;
r_1 = 0;
a_1 = 0;
d_1 = 0;
r_2 = l_1;
a_2 = pi/2;
d_2 = 0;
r_3 = l_2;
a_3 = 0;
d_3 = 0;
r_4 = l_3;
a_4 = 0;
d_4 = 0;
r_5 = l_4;
a_5 = 0;
d_5 = 0;
t5 = 0;

% Transformation matrix and matrix multiplication to obtain end effector.
T01 = T_matrix(a_1, r_1, d_1, t1);
T12 = T_matrix(a_2, r_2, d_2, t2);
T23 = T_matrix(a_3, a_3, d_3, t3);
T34 = T_matrix(a_4, r_4, d_4, t4);
T45 = T_matrix(a_5, r_5, d_5, t5);
T05 = T01*T12*T23*T34*T45;
EE = T05*[0;0;0;1];

Px = EE(1);
Py = EE(2);
Pz = EE(3);

findJ = [diff(Px,t1), diff(Px,t2), diff(Px,t3), diff(Px,t4);
         diff(Py,t1), diff(Py,t2), diff(Py,t3), diff(Py,t4);
         diff(Pz,t1), diff(Pz,t2), diff(Pz,t3), diff(Pz,t4)];