function xx = estimator(uu, PP)
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
		[xhatdot, A, B] = f_dfdx_dfdu(xhat, u);
		Q = B*M*B';
		xhat = xhat + (Ts/N)*xhatdot;
		P = P + (Ts/N)*(A*P + P*A' + Q);
	end
	
	% correct	
	R = diag([PP.sigma_r.^2, PP.sigma_phi.^2]);
	norm_L = zeros(1, size(PP.landmarks, 2));
	for i = 1:size(PP.landmarks, 2)
		[yhat, C] = h_dhdx(xhat, u, PP, i);
		Li = P*C'/(R + C*P*C');
		%P = (eye(size(P, 1)) - Li*C)*P;
		P = P - Li*C*P;
		xhat = xhat + Li*(y(:, i) - yhat);
		norm_L(i) = norm(Li, 2);
	end
	norm_L = norm(norm_L, 2);
end
function [xdot, A, B] = f_dfdx_dfdu(xhat, u)
	[xdot, A] = numericalDiff(@(xhat) f(xhat, u), xhat);
	[~, B] = numericalDiff(@(u) f(xhat, u), u);
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