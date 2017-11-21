
function u = controller(params, t, x, xd)
  % x = current position
  % xd = current velocity

  % Use params.traj(t) to get the reference trajectory
  % e.g. (x - params.traj(t)) represents the instaneous trajectory error

  % params can be initialized in the initParams function, which is called before the simulation starts
  
  % SOLUTION GOES HERE -------------
  kp = 100; kd = 30;
  u = kp*(params.traj(t)-x) + kd*(-xd);
end