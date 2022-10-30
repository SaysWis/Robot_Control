function [x_up, P_up] = Kalman_Filter(x_k_1, P_k_1, y, F, H, Q, R)

x_k = F*x_k_1;
P_k = F*P_k_1*F'+Q;              % Project the State Covariance Ahead

Kk = P_k*H'*inv(H*P_k*H'+R);     % Kalman Gain
x_up = x_k+Kk*(y-H*x_k);         % Update Measurement with measurement
P_up = P_k-Kk*H*P_k;             % Update the Error Covariance

end