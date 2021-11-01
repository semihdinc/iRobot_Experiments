function params = solveEquations(A,B)
% This function solves the equations as Ax-B=0. Gets A and B, computes x
% parameters
% A :(mxn) Equation Matrix 
% B :(mx1) Constants Vector 
%
% params :(nx1) Parameters Vector

params = inv(A'*A)*(A'*B);