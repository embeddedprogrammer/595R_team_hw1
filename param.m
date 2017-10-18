P.x0 = -5;
P.y0 = -3;
P.theta0 = 90*pi/180;
P.sampleTime = 0.1;
P.Ts = P.sampleTime;
P.landmarks = [6 -7 6
	           4 8 -4];
%P.landmarks = [6
%	           4];
		   
P.sigma_r   = 0.1; %m
P.sigma_phi = 0.05; %rad
P.alpha_1 = 0.1;
P.alpha_2 = 0.01;
P.alpha_3 = 0.01;
P.alpha_4 = 0.1;