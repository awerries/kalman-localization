function [Q_matrix] = adapt_noise_covariance(Phi_matrix, P_new, P_old, Q_matrix, n, GNSS_epoch, corrections)
%% Adapt Q matrix to system noise
% Adjust measurement noise covariance matrix Q according to state correction sequence
% Adam Werries 2016, see Apache 2.0 license.

%% Mohamed and Schwarz method, 1999
k = GNSS_epoch-1;
if k > n
    C = zeros(15,15);
    for j = k-n+1:k
        C = C + corrections(:,j)*corrections(:,j)';
    end
    Q_matrix = C./n + P_new - Phi_matrix*P_old*Phi_matrix';
    Q_matrix = diag(diag(Q_matrix));
else
    
end
    
%% Ding and Wang method, 2007
% TODO


end