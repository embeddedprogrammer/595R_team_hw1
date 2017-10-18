function xx = estimator_pf(uu, PP)
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
	persistent x;
	persistent xhat;
	persistent P;
	persistent tLast;
	if t == 0
		% initialize particles by randomly sampling from a uniform distribution
		rnd = @(len, min, max)  min + rand(1, len)*(max - min);
		n_p = 25;
		x = [rnd(n_p, -10, 10); rnd(n_p, -10, 10); rnd(n_p, -pi, pi)];
		[xhat, P] = compute_covariance(x);
	end
	y = reshape(y, 2, n);
	Ts = t - tLast;
	if Ts > 0 %Not sure why simulink is sampling more than it should...
		[x, xhat, P] = particleFilter(x, P, y, u, PP, Ts);
		draw_particles(x, tLast);
	end
	
	tLast = t;
	norm_L = 0;
	xx = [xhat; diag(P); norm_L];
end
function [x, xhat, P] = particleFilter(x, P, y, u, PP, Ts)
	% predict using noise model
    v     = u(1);
    omega = u(2);
	M = diag([(PP.alpha_1*abs(v) + PP.alpha_2*abs(omega)).^2, ... 
	         (PP.alpha_3*abs(v) + PP.alpha_4*abs(omega)).^2]);
	n = size(x, 2);
	for i = 1:n
		noise_u = [randn*sqrt(M(1, 1)); randn*sqrt(M(2, 2))];		 
		xdot = f(x(:, i), u + noise_u);
		x(:, i) = x(:, i) + xdot*Ts;
	end	
	
	% calculate weights for each x_i using w_i = P(z | x_i).
	w = zeros(n, 1);
	R = diag([PP.sigma_r.^2, PP.sigma_phi.^2]);	
	for i = 1:n
		S = R; %is this right?
		for j = 1:size(PP.landmarks, 2)
			yhat = h(x(:, i), 0, PP, j);
			d = size(x, 1);
			mahalDistSqr = (y(:, j) - yhat)'*inv(S)*(y(:, j) - yhat);
			logpdf = -1/2*log((2*pi)^d*det(R)) - 1/2*mahalDistSqr;
			%pdf = 1/sqrt((2*pi)^d*det(R))*exp(-1/2*mahalDistSqr);
			%pdf = mvnpdf(y(:, j)', yhat', S);
			w(i) = w(i) + logpdf;
		end
	end
	
	% normalize weights
	w = w - max(w);
	w = exp(w);
	w = w / sum(w);
	
	% resample using low-variance random sampler (spin wheel only once)
	c = 0; %cumulative sum, ie. c = sum(w(1:i))
	idxs = zeros(n, 1);
	j = 1; %
	c_f = rand/n; % cumulative value to find
	for i = 1:n
		c = c + w(i);
		while c > c_f
			idxs(j) = i;
			j = j + 1;
			c_f = c_f + 1/n;
		end
	end
	x = x(:, idxs);
	
	% add noise to samples based on covariance of original distribution
	% to ensure covariance doesn't become overconfident because the 
	% sample density is too low
	P_noise = P/n^(1/d);
	noise = mvnrnd(zeros(3, 1)', P_noise, n)';
	x = x + noise;
	
	% calculate new covariance
	[xhat, P] = compute_covariance(x);
end
function [mean, covariance] = compute_covariance(x)
	mean = sum(x, 2) / size(x, 2);
	x_offset = x - mean;
	covariance = x_offset*x_offset' / size(x, 2);
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