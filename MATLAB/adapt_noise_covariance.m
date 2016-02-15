function [R_matrix] = adapt_noise_covariance(H_matrix, P_matrix, R_matrix, n, residuals)
%% Adapt R matrix to sensor noise
% Adjust measurement noise covariance matrix R according to residual sequence
% Adam Werries 2016, see Apache 2.0 license.
k = size(residuals,2);
if k > n
    C = zeros(6,6);
    for j = k-n:k
        C = C + residuals(:,j)*residuals(:,j)';
    end
    C = C./n;
    R_matrix = C - H_matrix*P_matrix*H_matrix';
%     innovations = innovations(:,ceil(n/2):end);
end