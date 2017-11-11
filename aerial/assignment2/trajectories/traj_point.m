function desired_state = traj_point(t, ~)
initial_pos = [0; 1];

if t <1
    desired_state.pos = initial_pos;
    desired_state.vel = [0;0];
    desired_state.acc = [0;0];
else
    desired_state.pos = initial_pos + [1;0];
    desired_state.vel = [0;0];
    desired_state.acc = [0;0];
end

end
