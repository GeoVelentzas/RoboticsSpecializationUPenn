function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    % Check if the first time running this function
    dt = t - previous_t;
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    Af = [1 0 0.33 0; 0 1 0 0.33; 0 0 1 0; 0 0 0 1];
    C = [1 0 0 0; 0 1 0 0];
    Sm = diag([0.01 0.001 0.1 0.01]);
    R = diag([0.001, 0.001]);
    
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    
%% KALMAN FILTER %%
    P = param.P;
    
    P = A*P*A' + Sm;
    K = P*C'*inv(R+C*P*C');
    z = [x y]';
    xhat = A*state' + K*(z - C*A*state');
    prediction = Af*xhat;
    predictx = prediction(1);
    predicty = prediction(2);
    state = xhat';
    P = P-K*C*P;
    
    param.P = P;
%     %% TODO: Add Kalman filter updates
%     % As an example, here is a Naive estimate without a Kalman filter
%     % You should replace this code
%     vx = (x - state(1)) / (t - previous_t);
%     vy = (y - state(2)) / (t - previous_t);
%     % Predict 330ms into the future
%     predictx = x + vx * 0.330;
%     predicty = y + vy * 0.330;
%     % State is a four dimensional element
%     state = [x, y, vx, vy];
end
