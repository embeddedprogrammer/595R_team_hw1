function yy = unicycle_sensors(uu, P)
	% process inputs to function
	xy      = uu(1:2);
	theta   = uu(3);
 	%t       = uu(4);
	
	dxy = P.landmarks - xy; %Matlab 2016b performs implicit expansion (equivalent of Numpy broadcasting)
	range = sqrt(sum(dxy.^2, 1));
	angle = atan2(dxy(2, :), dxy(1, :)) - theta;
	range = range + randn(1, size(P.landmarks, 2))*P.sigma_r;
	angle = angle + randn(1, size(P.landmarks, 2))*P.sigma_phi;
	yy = reshape([range; angle], [], 1);
end