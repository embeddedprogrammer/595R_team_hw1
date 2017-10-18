function u = controller(uu, P)
	% process inputs to function
	xhat    = uu(1:3);
 	t       = uu(4);
	v     = 1 + 0.5*cos(2*pi*0.2*t);
	omega = -0.2 + 2*cos(2*pi*0.6*t);
	u = [v; omega];
end