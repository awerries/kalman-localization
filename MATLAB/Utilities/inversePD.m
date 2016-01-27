function [ A_ ] = inversePD( A )
%INVERSEPD Computes the inv of a matrix using Cholesky decomposition
%   A is expected to be positive definite

M = size(A,1);
[R, b] = chol(A);
if b~=0
    disp('Input matrix is not positive definite!');
    A = A
    return
end
R_ = R \ eye(M);
A_ = R_ * R_';
end

