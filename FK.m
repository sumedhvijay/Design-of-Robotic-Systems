function position = FK(t1, t2, t3, t4)

pi = sym('pi');
%Takes user input
t_1 = t1;
t_2 = t2;
t_3 = t3;
t_4 = t4;
t_5 = 0;

% Follwing are the (approximate) D-H parameters.
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
t_5 = 0;
%Transformation matrix of each joint.
T_1 = T_matrix(a_1, r_1, d_1, t_1);
T_2 = T_matrix(a_2, r_2, d_2, t_2);
T_3 = T_matrix(a_3, r_3, d_3, t_3);
T_4 = T_matrix(a_4, r_4, d_4, t_4);
T_5 = T_matrix(a_5, r_5, d_5, t_5);
%Matrix Multiplication to get transformation matrix of end effector(EE).
T_5 = T_1*T_2*T_3*T_4*T_5;
EE = T_5*[0;0;0;1];
%Show output.
digits(6);
position = vpa(subs(EE));
position = position(1:3);

end