function yy = h(xhat, u, P, i)
	xy      = xhat(1:2);
	theta   = xhat(3);
	dxy = P.landmarks(:, i) - xy;
	range = sqrt(sum(dxy.^2));
	angle = atan2(dxy(2), dxy(1)) - theta;
	yy = [range; angle];
end