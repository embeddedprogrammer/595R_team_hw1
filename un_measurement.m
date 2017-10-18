function loc = h(xhat, y, P, i)
	range = y(1);
	angle = xhat(3) + y(2);
	dxy = range*[cos(angle); sin(angle)];
	loc = xhat(1:2) + dxy;
end