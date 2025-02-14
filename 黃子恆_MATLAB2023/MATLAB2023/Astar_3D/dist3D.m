function dist = dist3D(P1,P2)
% This function calculates the distance between any two cartesian 
% coordinates. Three dimension.
% All functions used are supported fo C\C++ Code Generation
x1=P1(1);
y1=P1(2);
z1=P1(3);
x2=P2(1);
y2=P2(2);
z2=P2(3);
dist=sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
