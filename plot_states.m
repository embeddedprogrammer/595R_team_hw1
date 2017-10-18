function plot_states()
	x        = uu(1);
	y        = uu(2);
	theta    = 180/pi*uu(3);
	xhat     = uu(4);
	yhat     = uu(5);
	thetahat = 180/pi*uu(6);
	x_err    = abs(x - xhat);
	y_err    = abs(y - yhat);
	theta_err = abs(theta - thetahat);
	
	x_2sigma      = sqrt(uu(7))*2;
	y_2sigma      = sqrt(uu(8))*2;
	theta_2sigma  = 180/pi*sqrt(uu(9))*2;
	L             = uu(10);
	t             = uu(11);
    
	% define persistent variables 
	persistent x_handle
	persistent y_handle
	persistent theta_handle
	persistent x_err_handle
	persistent y_err_handle
	persistent theta_err_handle
	persistent L_handle

	% first time function is called, initialize plot and persistent vars
	if t == 0
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
		x_err_handle  = [];
		y_err_handle  = [];
		theta_err_handle  = [];
		L_handle  = [];
	end

	% at every time step, redraw state variables
	x_handle      = plot_vars([4 2 1], t, [x, xhat], 'x', x_handle);
	x_err_handle  = plot_vars([4 2 2], t, [x_err, x_2sigma], 'err_x', x_err_handle);
	y_handle      = plot_vars([4 2 3], t, [y, yhat], 'y', y_handle);
	y_err_handle  = plot_vars([4 2 4], t, [y_err, y_2sigma], 'err_y', y_err_handle);
	theta_handle  = plot_vars([4 2 5], t, [theta, thetahat], '\theta', theta_handle);
	theta_err_handle  = plot_vars([4 2 6], t, [theta_err, theta_2sigma], 'err_\theta', theta_err_handle);
	L_handle  = plot_vars([4 2 8], t, [L], 'L_norm', L_handle);
end

% Remember position when closed
function gcfClose(~, ~)
	pos = get(gcf, 'Position');
	assignin('base', 'plotWindowPos', pos);
	delete(gcf);
end

% Plot variables in blue, green, and red. Label. Set click to expand.
function handle = plot_vars(s, t, y, ylab, handle)
	if isempty(handle)
		subplot(s(1), s(2), s(3));
		hold on;
		lineOptions = {'b', 'g--', 'r-.'};
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