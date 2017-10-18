function draw_particles(x, t)
	% define persistent variables 
	persistent scatter_plot_handle;

	% first time function is called, initialize plot
	if t == 0
		figure(1)
		scatter_plot_handle = scatter_plot(x(1, :), x(2, :), []);

    % at every other time step, redraw particles
	else
		scatter_plot(x(1, :), x(2, :), scatter_plot_handle);
	end
end
function handle = scatter_plot(x, y, handle)
	if isempty(handle)
		handle = plot(x, y, '.');
	else
		set(handle, 'XData', x, 'YData', y);
	end
end

