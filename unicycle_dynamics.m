function [sys,x0,str,ts,simStateCompliance] = unicycle_dynamics(t,x,u,flag,P)
    switch flag
        case 0
            [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(P);
        case 1
            sys = mdlDerivatives(t,x,u,P);
        case 2
            sys = mdlUpdate(t,x,u);
        case 3
            sys = mdlOutputs(t,x,u,P);
        case 4
            sys = mdlGetTimeOfNextVarHit(t,x,u);
        case 9
            sys = mdlTerminate(t,x,u);
        otherwise % Unexpected flags
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end

% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(P)
    % call simsizes for a sizes structure, fill it in and convert it to a sizes array.
    sizes = simsizes;

    sizes.NumContStates  = 3;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 3;
    sizes.NumInputs      = 2;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1; % at least one sample time is needed

    sys = simsizes(sizes);

    % initialize the initial conditions
    x0  = [P.x0; P.y0; P.theta0];

    % str is always an empty matrix
    str = [];

    % initialize the array of sample times
    ts  = [0 0];

    % Specify the block simStateCompliance. The allowed values are:
    %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
    %    'DefaultSimState', < Same sim state as a built-in block
    %    'HasNoSimState',   < No sim state
    %    'DisallowSimState' < Error out when saving or restoring the model sim state
    simStateCompliance = 'UnknownSimState';
end

% mdlDerivatives
% Return the derivatives for the continuous states.
% t - time
% x - continuous state (altered by time passage)
% u - discrete state (input to s-function block)
% P - additional parameters
% sys output - rate of change of each state (qdot and qddot)
function sys = mdlDerivatives(t,x,u,P)
%	xx     = x(1);
%	yy     = x(2);
	theta = x(3);	
	v     = u(1);
	omega = u(2);
	
	% add noise on control input
	M = diag([(P.alpha_1*abs(v) + P.alpha_2*abs(omega)).^2, ... 
	         (P.alpha_3*abs(v) + P.alpha_4*abs(omega)).^2]);
	v = v + randn*sqrt(M(1, 1));
	omega = omega + randn*sqrt(M(2, 2));
	
	% calc derivatives
	xdot = v*cos(theta);
	ydot = v*sin(theta);
	thetadot = omega;
	sys = [xdot; ydot; thetadot];
end

% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step requirements.
function sys = mdlUpdate(t,x,u)
    sys = [];
end

% mdlOutputs
% Return the block outputs.
function sys = mdlOutputs(t,x,u,P)
    sys = x;
end

% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
function sys = mdlGetTimeOfNextVarHit(t,x,u)
    sampleTime = 1;    %  Example, set the next hit to be one second later.
    sys = t + sampleTime;
end

% mdlTerminate
% Perform any end of simulation tasks.
function sys = mdlTerminate(t,x,u)
    sys = [];
end
