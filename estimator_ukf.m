function xx = estimator_ukf(uu, PP)
	try
		xx = estimator_(uu, PP);
	catch e
		msgString = getReport(e);
		fprintf(2,'%s\n', msgString);
		rethrow(e);
	end
end
function xx = estimator_(uu, PP)
	% process inputs to function
	n = size(PP.landmarks, 2);
	y       = uu(1:(n*2));
	u       = uu((n*2)+1:(n*2)+2);
	t       = uu((n*2)+3);
	persistent xhat;
	persistent P;
	persistent tLast;
	persistent norm_L;
	if t == 0
		xhat = [PP.x0; PP.y0; PP.theta0];
		P = diag([1, 1, 1].^2);
		xhat(1) = xhat(1) + randn*sqrt(P(1, 1));
		xhat(2) = xhat(3) + randn*sqrt(P(2, 2));
		xhat(2) = xhat(3) + randn*sqrt(P(3, 3));
		tLast = 0;
		norm_L = 0;
	end
	y = reshape(y, 2, n);
	Ts = t - tLast;
	if Ts > 0 %Not sure why simulink is sampling more than it should...
		[xhat, P, norm_L] = extendedKalmanFilter(xhat, P, y, u, PP, Ts);
	end
	tLast = t;
	xx = [xhat; diag(P); norm_L];
end
function [xhat, P, norm_L] = extendedKalmanFilter(xhat, P, y, u, PP, Ts)
	% predict
    v     = u(1);
    omega = u(2);
	M = diag([(PP.alpha_1*abs(v) + PP.alpha_2*abs(omega)).^2, ... 
	         (PP.alpha_3*abs(v) + PP.alpha_4*abs(omega)).^2]);
	N = 10;
	for i = 1:N
		B = dfdu(xhat, u);
		Q = B*M*B';

%%%% METHOD 1 (Easy simple version)
		% propogate w/out adding process noise
		[xhat_sp, xhat_wm, xhat_wc] = ukf_draw_sigma_points(xhat, P);
		for j = 1:size(xhat_sp, 2)
			xhatdot = f(xhat_sp(:, j), u);
			xhat_sp(:, j) = xhat_sp(:, j) + (Ts/N)*xhatdot;
		end
		[xhat, P] = ukf_compute_covariance(xhat_sp, xhat_wm, xhat_wc);
		
		% add process noise
		P = P + (Ts/N)*Q;

%%%% METHOD 2 (Try for each sigma points)
% 		% propogate with adding process noise
% 		[xhat_sp, xhat_wm, xhat_wc] = ukf_draw_sigma_points(xhat, P);
% 		[u_sp, u_wm, u_wc] = ukf_draw_sigma_points(u, M);
% 		
% 		% for each xhat sigma point
% 		for j = 1:size(xhat_sp, 2)
% 			% for current xhat sigma point, get average xhatdot (by iterating through u sigma points)
% 			for k = 1:size(u_sp, 2)
% 				xhatdot_sp(:, k) = f(xhat_sp(:, j), u_sp(:, k));
% 			end
% 			[xhatdot, P_(:, :, j)] = ukf_compute_covariance(xhatdot_sp, u_wm, u_wc);
% 			
% 			% apply xhatdot to xhat sigma point (using euler integration)
% 			xhat_sp(:, j) = xhat_sp(:, j) + (Ts/N)*xhatdot;
% 		end
% 		[xhat, P] = ukf_compute_covariance(xhat_sp, xhat_wm, xhat_wc);
% 		P = P + ukf_compute_covariance2(P_, xhat_wc);
	
%%%% METHOD 3 (Pair sigma points with all combinations)
% 		% propogate with adding process noise
% 		[xhat_sp, xhat_wm, xhat_wc] = ukf_draw_sigma_points(xhat, P);
% 		[u_sp, u_wm, u_wc] = ukf_draw_sigma_points(u, M);
% 		[xhat_sp_, u_sp_, wm_, wc_] = combine_sigma_points(xhat_sp, xhat_wm, xhat_wc, u_sp, u_wm, u_wc);
% 		
% 		% for each xhat sigma point
% 		for j = 1:size(xhat_sp_, 2)
% 			xhatdot = f(xhat_sp_(:, j), u_sp_(:, j));
% 			xhat_sp_(:, j) = xhat_sp_(:, j) + (Ts/N)*xhatdot;
% 		end
% 		[xhat, P] = ukf_compute_covariance(xhat_sp_, wm_, wc_);
% 		P = P + (Ts/N)*Q;
	end
	
	% correct
	R = diag([PP.sigma_r.^2, PP.sigma_phi.^2]);
	norm_L = zeros(1, size(PP.landmarks, 2));
	for i = 1:size(PP.landmarks, 2)
		% get measurement mean, covariance, and cross covariance
		[xhat_sp, xhat_wm, xhat_wc] = ukf_draw_sigma_points(xhat, P);
		yhat_sp = zeros(2, size(xhat_sp, 2));
		for j = 1:size(xhat_sp, 2)
			yhat_sp(:, j) = h(xhat_sp(:, j), u, PP, i);
		end
		[~, yhat, ~, CPCt, PCt] = ukf_compute_cross_covariance(xhat_sp, yhat_sp, xhat_wm, xhat_wc);
		
		% Calculate KF gain
		S = R + CPCt;
		L = PCt/S;
		xhat = xhat + L*(y(:, i) - yhat);
		P = P - L*S*L';
		norm_L(i) = norm(L, 2);
	end
	norm_L = norm(norm_L, 2);
end
function [sigma_points, w_m, w_c] = ukf_draw_sigma_points(mean, covariance, U)
	% parameters
	U.kappa = 0;
	U.alpha = 1;
	U.beta = 2;

	% find sigma points
	n = size(mean, 1);
	lambda = U.alpha.^2*(n + U.kappa) - n;
	gamma = n + lambda;
	P_sqrt = chol(covariance)';
	sigma_points = [mean, mean + sqrt(gamma)*P_sqrt, mean - sqrt(gamma)*P_sqrt];
	
	% compute weights
	w_m = [lambda/(n+lambda),                  ones(1, 2*n)./(2*(n + lambda))];
	w_c = [w_m(1) + (1 - U.alpha.^2 + U.beta), ones(1, 2*n)./(2*(n + lambda))];	
end
function [mean, covariance] = ukf_compute_covariance(sigma_points, w_m, w_c)
	mean = sum(sigma_points.*w_m, 2);
	sigma_offset = sigma_points - mean;
	covariance = sigma_offset*diag(w_c)*sigma_offset';
end
function [covariance] = ukf_compute_covariance2(covariances, w_c)
	covariance = zeros(size(covariances, 1), size(covariances, 2));
	for i = 1:length(size(covariances, 3))
		covariance = covariance + covariances(:, :, i)*w_c(i);
	end	
end
function [sp1_, sp2_, wm_, wc_] = combine_sigma_points(sp1, wm1, wc1, sp2, wm2, wc2)
	% create sigma points by using all possible combinations.
	n1 = size(sp1, 2);
	n2 = size(sp2, 2);
	sp1_ = zeros(size(sp1, 1), n1*n2);
	sp2_ = zeros(size(sp2, 1), n1*n2);
	wm_ = zeros(1, n1*n2);
	wc_ = zeros(1, n1*n2);
	k = 1;
	for i = 1:n1
		for j = 1:n2
			sp1_(:, k) = sp1(:, i);
			sp2_(:, k) = sp2(:, j);
			wm_(k) = wm1(i)*wm2(j);
			wc_(k)= wc1(i)*wc2(j);			
			k = k + 1;
		end
	end
end
function [mean1, mean2, covariance1, covariance2, covariance12] = ukf_compute_cross_covariance(sigma_points1, sigma_points2, w_m, w_c)
	mean1 = sum(sigma_points1.*w_m, 2);
	mean2 = sum(sigma_points2.*w_m, 2);
	sigma_offset1 = sigma_points1 - mean1;
	sigma_offset2 = sigma_points2 - mean2;
	covariance1 = sigma_offset1*diag(w_c)*sigma_offset1';
	covariance2 = sigma_offset2*diag(w_c)*sigma_offset2';
	covariance12 = sigma_offset1*diag(w_c)*sigma_offset2';
end
function B = dfdu(xhat, u)
	[~, B] = numericalDiff(@(u) f(xhat, u), u);
end
function [fcn0, J] = numericalDiff(fcn, x)
	h = .0001;
	fcn0 = fcn(x);
	J = zeros(length(fcn0), length(x));
	for i = 1:length(x)
		dx = zeros(size(x));
		dx(i) = h;
		J(:, i) = (fcn(x + dx) - fcn0) / h;
	end
end
function xhatdot = f(xhat, u)
    theta = xhat(3);
    v     = u(1);
    omega = u(2);
	xdot = v*cos(theta);
	ydot = v*sin(theta);
	thetadot = omega;
    xhatdot = [xdot; ydot; thetadot];
end
function yy = h(xhat, u, P, i)
	xy      = xhat(1:2);
	theta   = xhat(3);
	dxy = P.landmarks(:, i) - xy;
	range = sqrt(sum(dxy.^2));
	angle = atan2(dxy(2), dxy(1)) - theta;
	yy = [range; angle];
end