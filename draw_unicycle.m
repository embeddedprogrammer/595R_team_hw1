function draw_unicycle(uu, P)
	% process inputs to function
	x       = uu(1);     
	y       = uu(2);
	theta   = uu(3);
	t       = uu(4);

	% define persistent variables 
	persistent circle_handle;
	persistent line_handle;
	r = .7;

	% first time function is called, initialize plot and persistent vars
	if t == 0
		figure(1)
		clf
		circle_handle = drawCircle(x, y, r, []);
		hold on
		line_handle = drawLine(x, y, theta, r, []);
		drawLandmarks(P.landmarks(1, :), P.landmarks(2, :))
		grid on
		axis([-10 10 -10 10]);

    % at every other time step, redraw spring and box
	else
		circle_handle = drawCircle(x, y, r, circle_handle);
		line_handle = drawLine(x, y, theta, r, line_handle);
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

