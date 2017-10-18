clear vars;
param;
load('hw2_soln_data.mat');

for i = 1:length(t)
	u_ = [v(i); om(i)];
	x_ = [x(i); y(i); th(i)];
	t_ = t(i);
	y_ = unicycle_sensors([x_; t_], P);
	uu_ = estimator([y_; u_; t_], P);
	xhat_ = uu_(1:3);
	P_diag_ = uu_(4:6);
	norm_L_ = uu_(7);
	plot_states([x_; xhat_; P_diag_; norm_L_; t_]);
end
