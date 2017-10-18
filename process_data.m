load('processed_data');
param;
l_idx = 1;
rst_meas = true;
for i = 1:length(odom_t)
	t = odom_t(i);
	u = vel_odom(:, i);
	
	% propogate
	y = nan(2, 11);
	rst = (i == 1);
	[x, xhat, P_] = estimator_pf(y, u, t, rst, P);
	
	% update
	if i < length(odom_t) && l_idx <= length(l_time)
		while l_time(l_idx) > odom_t(i) && l_time(l_idx) < odom_t(i + 1)
			y = [l_depth(:, l_idx)'; -l_bearing(:, l_idx)'];
			[x, xhat, P_] = estimator_pf(y, u, t, 0, P);
			l_idx = l_idx + 1;
		end
	end
	
	% plot results
	draw_unicycle(xhat, rst, P);
	draw_particles(x, rst);
	if any(any(~isnan(y)))
		draw_measurements(xhat, y, rst_meas, P)
		rst_meas = false;
	end
end