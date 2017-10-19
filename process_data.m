param;
load('processed_data');

% remove outliers
% There are 7 measurements which have depth of less than 6mm. This is clearly impossible.
%bad_idxs = l_depth < 0.01;
%l_depth(bad_idxs) = nan;
%l_bearing(bad_idxs) = nan;
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
	while i < length(odom_t) && l_idx <= length(l_time) && ...
			l_time(l_idx) > odom_t(i) && l_time(l_idx) < odom_t(i + 1)
		y = [l_depth(:, l_idx)'; -l_bearing(:, l_idx)'];
		[x, xhat, P_, err, mahalDist, unique_samples] = estimator_pf(y, u, t, 0, P);
		l_idx = l_idx + 1;
	end
	
	% plot results
	plot_states(xhat, P_, t, rst);
	draw_unicycle(xhat, rst, P);
	draw_particles(x, rst);
	if any(any(~isnan(y)))
		draw_measurements(xhat, y, rst_meas, P)
		plot_measurements(t, rst_meas, err, mahalDist, unique_samples)
		rst_meas = false;
	end
end