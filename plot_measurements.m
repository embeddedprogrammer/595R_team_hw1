function plot_measurements(t, rst, err, mahalDist, unique_samples)
	% define persistent variables 
	persistent range_handle bearing_handle unique_handle mahal_handle
	range_err = err(1);
	bearing_err = err(2)*180/pi;

	% first time function is called, initialize plot and persistent vars
	if rst
		% Create figure that remembers its position
		if ishandle(3)
			figure(3)
			clf
		elseif evalin('base', 'exist(''plotWindowPos2'')')
			figure(3)
			pos = evalin('base', 'plotWindowPos2');
			set(gcf, 'Position', pos);
		else
			figure(3)
		end
		set(gcf, 'CloseRequestFcn', @gcfClose);
		
		% Reset handle persistent vars
		range_handle  = [];
		bearing_handle = [];
		mahal_handle = [];
		unique_handle = [];
	end

	% at every time step, redraw state variables
	range_handle = plot_vars([4 1 1], t, range_err, 'range', range_handle, {'r'});
	bearing_handle = plot_vars([4 1 2], t, bearing_err, 'bearing', bearing_handle, {'r'});
	mahal_handle = plot_vars([4 1 3], t, mahalDist, 'mahal', mahal_handle, {'r'});
	unique_handle = plot_vars([4 1 4], t, unique_samples, 'unique', unique_handle, {'r'});
end

% Remember position when closed
function gcfClose(~, ~)
	pos = get(gcf, 'Position');
	assignin('base', 'plotWindowPos2', pos);
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