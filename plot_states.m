function plot_states(xhat, P, t, rst)
	x        = xhat(1);
	y        = xhat(2);
	theta    = 180/pi*xhat(3);
	x_2sigma      = sqrt(P(1, 1))*2;
	y_2sigma      = sqrt(P(2, 2))*2;
	theta_2sigma  = 180/pi*sqrt(P(3, 3))*2;
    
	% define persistent variables 
	persistent x_handle y_handle theta_handle x2_handle y2_handle theta2_handle

	% first time function is called, initialize plot and persistent vars
	if rst
		% Create figure that remembers its position
		if ishandle(2)
			figure(2)
			clf
		elseif evalin('base', 'exist(''plotWindowPos'')')
			figure(2)
			pos = evalin('base', 'plotWindowPos');
			set(gcf, 'Position', pos);
		else
			figure(2)
		end
		set(gcf, 'CloseRequestFcn', @gcfClose);
		
		% Reset handle persistent vars
		x_handle      = [];
		y_handle      = [];
		theta_handle  = [];
		x2_handle      = [];
		y2_handle      = [];
		theta2_handle  = [];
	end

	% at every time step, redraw state variables
	x_handle      = plot_w_bounds([3 2 1], t, x, x_2sigma, 'x', x_handle);
	y_handle      = plot_w_bounds([3 2 3], t, y, y_2sigma, 'y', y_handle);
	theta_handle  = plot_w_bounds([3 2 5], t, theta, theta_2sigma, '\theta', theta_handle);
	x2_handle = plot_vars([3 2 2], t, [x_2sigma], 'x', x2_handle, {'r-.'});
	y2_handle = plot_vars([3 2 4], t, [y_2sigma], 'y', y2_handle, {'r-.'});
	theta2_handle = plot_vars([3 2 6], t, [theta_2sigma], '\theta', theta2_handle, {'r-.'});
end

% Remember position when closed
function gcfClose(~, ~)
	pos = get(gcf, 'Position');
	assignin('base', 'plotWindowPos', pos);
	delete(gcf);
end

function handle = plot_w_bounds(s, t, y, y_2sigma, ylab, handle)
	handle = plot_vars(s, t, [y, y - y_2sigma, y + y_2sigma], ylab, handle, {'b', 'r-.', 'r-.'});
end

% Plot variables in blue, green, and red. Label. Set click to expand.
function handle = plot_vars(s, t, y, ylab, handle, lineOptions)
	if isempty(handle)
		subplot(s(1), s(2), s(3));
		hold on;
		if nargin < 6
			lineOptions = {'b', 'g--', 'r-.'};
		end
		for i = 1:length(y)
			handle(i) = plot(t, y(i), lineOptions{i});
		end
		ylabel(ylab)
		set(get(gca, 'YLabel'), 'Rotation', 0.0);
		set(gca, 'ButtonDownFcn', @gcaExpand);
	else
		for i = 1:length(y)
			set(handle(i), 'Xdata', [get(handle(i), 'Xdata'), t]);
			set(handle(i), 'Ydata', [get(handle(i), 'Ydata'), y(i)]);
		end		
	end
end

% Create a callback that expands the subplot when clicked.
function gcaExpand(~, ~)
	set(copyobj(gca, uipanel('Position', [0 0 1 1])), 'Units', 'normal', 'OuterPosition', [0 0 1 1], 'ButtonDownFcn', 'delete(get(gca, ''Parent''))');
end