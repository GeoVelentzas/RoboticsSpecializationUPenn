
function u = controller(params, t, phi, phidot)


%   syms th phi dth dphi u
%   qddot = eom(params, th, phi, dth, dphi, u);
%   phiddot = qddot(2);
%   phiddot = subs(phiddot, {phi, phidot}, {0,0});
%   these will return that phiddot = -60000*u
  % STUDENT FILLS THIS OUT
  % 
  % Initialize any added state like this:
%   % 
  persistent ei
  if isempty(ei)
    % initialize
    ei = 0;
  end
  persistent time
  if isempty(time)
    % initialize
    time = t;
  end
  
  dt = t - time;
  time = t;

  
  kp = 40;
  kd = 0.5;
  ki = 1000;
  ei = ei + (0-phi*dt); 
  
  u = kp*(0-phi) + kd*(0-phidot) + ki*(ei);
%   u = 1/60000*phiddot;
  u= -u;
%   disp(t)
%   figure(1); 
%   subplot(2,1,1);
%   plot(t, phi, '*');hold on;
%   subplot(2,1,2);
%   plot(t, phidot, 'o'); hold on;
%   drawnow;
end


















