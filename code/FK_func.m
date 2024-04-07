function[T]=FK_func(theta)
%DH table
%alpha a d theta 
DH=[pi/2,0,0.1625,theta(1);
    0,-0.425,0,theta(2);
    0,-0.3922,0,theta(3);
    pi/2,0,0.1333,theta(4);
    -pi/2,0,0.0997,theta(5);
    0,0,0.00996,theta(6)];

T=eye(4);
for i = 1:size(DH, 1)
    T=T*A_mtx(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
end

end

function[Ai0_i1]=A_mtx(alpha,a,d,theta)
A_int_i0=[1,0,0,a;
         0,cos(alpha),-sin(alpha),0;
         0,sin(alpha),cos(alpha),0;
         0,0,0,1];

A_i1_int=[cos(theta),-sin(theta),0,0;
          sin(theta),cos(theta),0,0;
          0,0,1,d;
          0,0,0,1];

Ai0_i1=A_i1_int*A_int_i0;
% Ai0_i1 = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
%            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
%            0, sin(alpha), cos(alpha), d;
%            0, 0, 0, 1];
end