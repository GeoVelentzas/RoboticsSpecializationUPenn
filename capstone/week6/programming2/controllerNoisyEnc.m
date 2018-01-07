
function u = controllerNoisyEnc(params, t, obs, th, dth)
    
  r = params.r;
  
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);

  x = r*(th+phi);
  dx = r*(dth+phidot);
  
  kp1 = 0.15;
  kd1 = 0.15;
  u1 = kp1*(params.traj(t)-x) + kd1*(0 - dx);
  phides = asin(u1);
  
  kp2 = 0.1;
  kd2 = 0.02;
  u2 = kp2*sin(phides - phi) + kd2*(0-phidot);
  
  u = -u2;
  
  
end








%%

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  
  persistent P xhat time2 k
  if isempty(P)
    P=1e3*eye(2);
    xhat = zeros(2,length(t));
    time2 = t;
    k = 1;
  end
  
  k = k+1;
  Q = diag([10000, 0.6]);
  R = diag([0.001, 0.01, 30]);
  dt = t - time2;
  time2 = t;
  A = [1 dt; 0 1];
  xhat(:,k) = A*xhat(:, k-1);
  P = A*P*A' + Q;
  H = [cos(xhat(1,k-1)) 0; -sin(xhat(1,k-1)) 0 ; 0 1];
  h = [sin(xhat(1,k-1)); cos(xhat(1,k-1)); xhat(2,k-1)];
  %       h = h + H*(xkp - xhat(:,k-1));
  K = P*H'*inv(H*P*H'+R);
  xhat(:,k) = xhat(:,k) + K*(z - h);
  P = (eye(2)-K*H)*P;
  xhatOut = xhat(:, k);
end

