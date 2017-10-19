param;
load('processed_data');
l_idx = 1;
rst_meas = true;
for i = 1:length(odom_t)
	t = odom_t(i);
	u = vel_odom(:, i);
	
	% propogate
	y = nan(2, 11);
	rst = (i == 1);
	[x, xhat, P_] = estimator_pf(y, u, t, rst, P);
	
	% plot results
	plot_states(xhat, P_, t, rst);
	draw_unicycle(xhat, rst, P);
	draw_particles(x, rst);
end