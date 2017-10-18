function draw_measurements(xhat, y, rst, P)
	% define persistent variables
	persistent scatter_plot_handle;
	
	% get position of measurements
	pts = nan(2, size(P.landmarks, 2));
	for i = 1:size(P.landmarks, 2)
		if ~any(isnan(y(:, i)))
			pts(:, i) = un_measurement(xhat, y(:, i), P, i);
		end
	end	

	% first time function is called, initialize plot
	if rst 
		figure(1)
		scatter_plot_handle = draw_all_meas(pts, P.landmarks, []);

    % at every other time step, redraw particles
	else
		draw_all_meas(pts, P.landmarks, scatter_plot_handle);
	end
	drawnow;
end
function handle = draw_all_meas(pts, landmarks, handle)
	if isempty(handle)
		handle = cell(size(landmarks, 2), 1);
	end
	for i = 1:size(landmarks, 2)
		handle{i} = draw_measurement(pts(:, i), landmarks(:, i), handle{i});
	end
end
function handle = draw_measurement(loc_hat, loc_actual, handle)
	if any(isnan(loc_hat))
		loc_hat = [0; 20];
		loc_actual = [0; 20];
	end
	star = [loc_hat];
	line = [loc_hat loc_actual];
	if isempty(handle)
		handle(1) = plot(star(1, :), star(2, :), 'r*');
		handle(2) = plot(line(1, :), line(2, :), 'r');
	else
		set(handle(1), 'XData', star(1, :), 'YData', star(2, :));
		set(handle(2), 'XData', line(1, :), 'YData', line(2, :));
	end
end
