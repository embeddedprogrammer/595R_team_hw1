function [x_, xhat_, P_, err, mahalDist, unique_samples] = estimator_pf(y, u, t, rst, PP)
	% process inputs to function
	n = size(PP.landmarks, 2);
	persistent x xhat P tLast;
	if rst
		% initialize particles by randomly sampling from a uniform distribution
		rnd = @(len, min, max)  min + rand(1, len)*(max - min);
		n_p = 1000;
		%x = PP.x0 + [rnd(n_p, -1, 1); rnd(n_p, -10, 10); rnd(n_p, -pi, pi)];
		x = [rnd(n_p, 0, 10); rnd(n_p, -1, 15); rnd(n_p, -pi, pi)];
		[xhat, P] = compute_covariance(x);
	end
	y = reshape(y, 2, n);
	Ts = t - tLast;
	[x, xhat, P, err, mahalDist, unique_samples] = particleFilter(x, P, y, u, PP, Ts);
	tLast = t;
	x_ = x;
	xhat_ = xhat;
	P_ = P;
end
function [x, xhat, P, errs, mahalDists, unique_samples] = particleFilter(x, P, y, u, PP, Ts)
	unique_samples = 0;
	mahalDists = 0;
	errs = [0; 0];
	% PREDICT
	if Ts > 0
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
	end	
	
	% MEASUREMENT UPDATE
	if any(any(~isnan(y)))
		% find out mahalanobis distance away
		% There might be a less hacky way to do this...
		[xhat, P] = compute_covariance(x);
		errs = [];
		mahalDists = [];
		R = diag([PP.sigma_r.^2, PP.sigma_phi.^2]);
		for i = 1:size(PP.landmarks, 2)
			if ~any(isnan(y(:, i)))
				%yhat = measurement(xhat, 0, PP, i);
				%R = diag([(y(1, i)*PP.alpha_r).^2, PP.sigma_phi.^2]);
				[yhat, C] = h_dhdx(xhat, u, PP, i);
				S = C*P*C' + R;
				err = angleMod(y(:, i) - yhat);
				errs = [errs abs(err)];
				mahalDist = sqrt(err'*inv(S)*err);
				if(mahalDist > 5)
					y(:, i) = nan;
				end
				mahalDists = [mahalDists sqrt(err'*inv(S)*err)];
			end
		end
		errs = mean(errs, 2);
		mahalDists = mean(mahalDists);

		% calculate weights for each x_i using w_i = P(z | x_i).
		n = size(x, 2);
		w = zeros(n, 1);
		for i = 1:n
			S = R; %is this right?
			for j = 1:size(PP.landmarks, 2)
				if ~any(isnan(y(:, j)))
					%R = diag([(y(1, j)*PP.alpha_r).^2, PP.sigma_phi.^2]);
					yhat = h(x(:, i), 0, PP, j);
					d = size(x, 1);
					err = angleMod(y(:, j) - yhat);
					mahalDistSqr = err'*inv(S)*err;
					logpdf = -1/2*log((2*pi)^d*det(R)) - 1/2*mahalDistSqr;
					%pdf = 1/sqrt((2*pi)^d*det(R))*exp(-1/2*mahalDistSqr);
					%pdf = mvnpdf(y(:, j)', yhat', S);
					w(i) = w(i) + logpdf;
				end
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
		unique_samples = length(unique(idxs));

		% add noise to samples based on covariance of original distribution
		% to ensure covariance doesn't become overconfident because the 
		% sample density is too low
		d = size(x, 1);
		P_noise = P/n^(1/d);
		noise = mvnrnd(zeros(3, 1)', P_noise, n)';
		x = x + noise;
	end
		
	% calculate new covariance
	[xhat, P] = compute_covariance(x);
end
function angleDiff = angleMod(angleDiff)
	angleDiff = mod(angleDiff + pi, 2*pi) - pi;
end
function [mean, covariance] = compute_covariance(x)
	mean = sum(x, 2) / size(x, 2);
	x_offset = x - mean;
	covariance = x_offset*x_offset' / size(x, 2);
end
function [yhat, C] = h_dhdx(xhat, u, P, i)
	[yhat, C] = numericalDiff(@(xhat) h(xhat, u, P, i), xhat);
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