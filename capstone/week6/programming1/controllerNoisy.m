
function u = controllerNoisy(params, t, obs)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for phi, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] with a* in units of g's, and gx in units of rad/s

  % This template code calls the function EKFupdate that you must complete below
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);

  % The rest of this function should ideally be identical to your solution in week 4
  % Student completes this
  
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

  
%   kp = 40;
%   kd = 0.5;
%   ki = 1000;
  
  kp = 40;
  kd = 3;
  ki = 1000;

%   kp = 54;
%   kd = 2.8;
%   ki = -2000;

  
  ei = ei + (0-phi*dt); 
  
  u = kp*(0-phi) + kd*(0-phidot) + ki*(ei);
%   u = -1/60000*phiddot;
   u= -u;
  
end

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.
  
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




% function xhatOut = EKFupdate(params, t, z)
% % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
% % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
% % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.
% 
% % Student completes this
% p0 = 1;
% sr = 0.1;
% sq = 0.009;
% persistent state P PT
% 
% if (t==0)
%     phi = atan2(z(1), z(2));
%     dphi = z(3);
%     state = [phi;dphi];
%     P = 1e3*eye(2);
%     PT = t;
%     xhatOut = state;
%     return;
% end
% 
% dt = t - PT;
% PT = t;
% 
% A = [1 dt; 0 1];
% W = [1 0; dt 1];
% % R = [sr 0 0; 0 sr 0; 0 0 sr];
% % Q = [sq 0; 0 sq];
% Q = diag([10000, 0.6]);
% R = diag([0.001, 0.01, 30]);
% 
% % Predict
% next_state = A*state;
% P_next = A*P*W+Q;
% 
% % Jacobian
% phi = atan2(z(1),z(2));
% H = [cos(phi) 0; -sin(phi) 0; 0 1];
% E = [cos(phi) -sin(phi) 0; 0 0 1];
% 
% % Kalman Gain
% KGain = P_next*E/(H*P_next*E+R);
% 
% state = state +KGain*(z - (H*next_state));
% P = ([1 0; 0 1] - (KGain*H))*P_next;
% xhatOut = state;
% end




