function [states_dot, y] = SwarmProblemDefinition(states, controls)
% States
x  = states(1);
y  = states(2);
vx = states(3);
vy = states(4);
time = states(5);

% Controls
ax = controls(1);
ay = controls(2);

% State derivatives
x_dot  = vx;
y_dot  = vy;
vx_dot = ax;
vy_dot = ay;
time_dot = 1;
states_dot = [x_dot; y_dot; vx_dot; vy_dot;time_dot];

% Outputs
x_out=x;
y_out=y;
vx_out=vx;
vy_out=vy;
time_out=time;
ax_out=ax;
ay_out=ay;
y = [x_out; y_out;vx_out;vy_out;time_out;ax_out;ay_out];
end