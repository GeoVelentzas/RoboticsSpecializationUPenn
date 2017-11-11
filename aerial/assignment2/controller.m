function [ F, M ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
% 10 10 140 17 180 80
Kpy = 10;
Kdy = 10;

Kpf = 140;
Kdf = 17;

Kpz = 180;
Kdz = 80;
 
phi_c = asin(-1/params.gravity*( des_state.acc(1) + Kdy*(des_state.vel(1)-state.vel(1))  +  Kpy*(des_state.pos(1)-state.pos(1))  ));

F = 1/cos(state.rot)*params.mass*(params.gravity + des_state.pos(2) + Kdz*(des_state.vel(2)-state.vel(2)) + Kpz*(des_state.pos(2)-state.pos(2)) );
M = params.Ixx*( Kdf*(0-state.omega)  +  Kpf*(phi_c-state.rot)  );

if F<params.minF
    F = params.minF;
end

if F>params.maxF
    F = params.maxF;
end

end

