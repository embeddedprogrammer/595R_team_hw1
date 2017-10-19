function draw_unicycle(xx, rst, P)
	% process inputs to function
	x       = xx(1);     
	y       = xx(2);
	theta   = xx(3);

	% define persistent variables 
	persistent circle_handle line_handle history_handle;
	r = .7;

	% first time function is called, initialize plot and persistent vars
	if rst
		figure(1)
		clf
		circle_handle = drawCircle(x, y, r, []);
		hold on
		line_handle = drawLine(x, y, theta, r, []);
		history_handle = plot_var(x, y, 'r', []);
		drawLandmarks(P.landmarks(1, :), P.landmarks(2, :))
		grid on
		axis([0 8 -4 20]);

    % at every other time step, redraw
	else
		drawCircle(x, y, r, circle_handle);
		drawLine(x, y, theta, r, line_handle);
		plot_var(x, y, 'r', history_handle);
	end
	drawnow	
end
function drawLandmarks(x, y)
	plot(x, y, 'bo');
end
function handle = drawLine(xc, yc, theta, r, handle)
	x = [0 cos(theta)*r] + xc;
	y = [0 sin(theta)*r] + yc;
	if isempty(handle)
		handle = plot(x, y, 'r');
	else
		set(handle, 'XData', x, 'YData', y);
	end
end

function handle = drawCircle(xc, yc, r, handle)
	theta = linspace(0, 2*pi, 50);
	x = cos(theta)*r + xc;
	y = sin(theta)*r + yc;
	if isempty(handle)
		handle = plot(x, y, 'r');
	else
		set(handle, 'XData', x, 'YData', y);
	end
end
function handle = plot_var(x, y, c, handle)
	if isempty(handle)
		handle = plot(x, y, c);
	else
		set(handle, 'Xdata', [get(handle, 'Xdata'), x]);
		set(handle, 'Ydata', [get(handle, 'Ydata'), y]);
	end
end

