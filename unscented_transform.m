% parameters
kappa = 0;
alpha = 1;
n = 2;
lambda = alpha.^2*(n + kappa) - n;
beta = 2;

% find sigma points
P = [1 .5; .5 1];
n = size(xhat, 1);
gamma = n + lambda;
P_sqrt = chol(P)';
sigma_points = [xhat, xhat + sqrt(gamma)*P_sqrt, xhat - sqrt(gamma)*P_sqrt];

% pass through function
plot(sigma_points(1, :), sigma_points(2, :), 'o')

% recompute covariance
w_m = [lambda/(n+lambda),              ones(1, 2*n)./(2*(n + lambda))];
w_c = [w_m(1) + (1 - alpha.^2 + beta), ones(1, 2*n)./(2*(n + lambda))];
xhat_new = sum(sigma_points.*w_m, 2);
sigma_offset = sigma_points - xhat_new;
P_new = sigma_offset*diag(w_c)*sigma_offset';