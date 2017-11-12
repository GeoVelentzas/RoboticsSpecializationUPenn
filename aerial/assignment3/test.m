clear all;

waypoints = [0    0   0;
              1    1   1;
              2    0   2;
              3    -1  1;
              4    0   0]';
    waypoints0 = waypoints;
    points = size(waypoints,2);
    number_of_paths = size(waypoints,2)-1;
    total_time = 15;
    
    for i = 1:number_of_paths
        distance(i) = norm(waypoints(:,i)-waypoints(:,i+1));
    end
    
    time = ceil(total_time*distance/sum(distance)); %time per path
    total_time = sum(time);
    
    for i = 1:number_of_paths
        S(i) = sum(time(1:i));
    end



C   = [           1           0             0              0             0              0              0              0     zeros(1,3*8);                                                                                                                       % p1 initial point1
                  1          S(1)         S(1)^2         S(1)^3        S(1)^4         S(1)^5         S(1)^6        S(1)^7   zeros(1,3*8);                                                                                                                       % p2 final interval point1
                  0           1             0              0             0              0              0              0     zeros(1,3*8);                                                                                                                       % 0 initial velocity zero
                  0           0             2              0             0              0              0              0     zeros(1,3*8);                                                                                                                       % 0 initial acc zero
                  0           0             0              6             0              0              0              0     zeros(1,3*8);                                                                                                                       % 0 initial jerk zero     
                  0           1           2*S(1)        3*S(1)^2      4*S(1)^3       5*S(1)^4      6*S(1)^5      7*S(1)^6     -0           -1           -2*S(1)        -3*S(1)^2      -4*S(1)^3       -5*S(1)^4      -6*S(1)^5      -7*S(1)^6   zeros(1,2*8) ;   % 0 1,2
                  0           0             2           6*S(1)        12*S(1)^2     20*S(1)^3      30*S(1)^4     42*S(1)^5    -0           -0             -2           -6*S(1)        -12*S(1)^2     -20*S(1)^3      -30*S(1)^4     -42*S(1)^5   zeros(1,2*8);   % 0 1,2
                  0           0             0             6           24*S(1)       60*S(1)^2     120*S(1)^3     210*S(1)^4   -0            -0             -0             -6           -24*S(1)       -60*S(1)^2     -120*S(1)^3     -210*S(1)^4 zeros(1,2*8);   % 0 1,2
                  0           0             0             0            24            120*S(1)     360*S(1)^2     840*S(1)^3   -0           -0             -0             -0            -24            -120*S(1)     -360*S(1)^2     -840*S(1)^3  zeros(1,2*8);   % 0 1,2
                  0           0             0             0             0              120        720*S(1)       2520*S(1)^2  -0           -0             -0             -0             -0              -120        -720*S(1)       -2520*S(1)^2 zeros(1,2*8);   % 0 1,2
                  0           0             0             0             0               0           720          5040*S(1)    -0           -0             -0             -0             -0               -0           -720          -5040*S(1)   zeros(1,2*8);   % 0 1,2
zeros(1,8)        1          S(1)         S(1)^2         S(1)^3        S(1)^4         S(1)^5         S(1)^6        S(1)^7    zeros(1,2*8);                                                                                                                       % p2 initial point 2
zeros(1,8)        1          S(2)         S(2)^2         S(2)^3        S(2)^4         S(2)^5         S(2)^6        S(2)^7    zeros(1,2*8);                                                                                                                       % p3 final point 2
zeros(1,8)        0           1           2*S(2)        3*S(2)^2      4*S(2)^3       5*S(2)^4      6*S(2)^5      7*S(2)^6     -0           -1           -2*S(2)        -3*S(2)^2      -4*S(2)^3       -5*S(2)^4      -6*S(2)^5      -7*S(2)^6    zeros(1,8);     % 0 2,3
zeros(1,8)        0           0             2           6*S(2)        12*S(2)^2     20*S(2)^3      30*S(2)^4     42*S(2)^5    -0           -0             -2           -6*S(2)        -12*S(2)^2     -20*S(2)^3      -30*S(2)^4     -42*S(2)^5   zeros(1,8);     % 0 2,3
zeros(1,8)        0           0             0             6           24*S(2)       60*S(2)^2     120*S(2)^3     210*S(2)^4   -0            -0             -0             -6           -24*S(2)       -60*S(2)^2     -120*S(2)^3     -210*S(2)^4 zeros(1,8);     % 0 2,3
zeros(1,8)        0           0             0             0            24            120*S(2)     360*S(2)^2     840*S(2)^3   -0           -0             -0             -0            -24            -120*S(2)     -360*S(2)^2     -840*S(2)^3  zeros(1,8);     % 0 2,3
zeros(1,8)        0           0             0             0             0              120        720*S(2)       2520*S(2)^2  -0           -0             -0             -0             -0              -120        -720*S(2)       -2520*S(2)^2 zeros(1,8);     % 0 2,3
zeros(1,8)        0           0             0             0             0               0           720          5040*S(2)    -0           -0             -0             -0             -0               -0           -720          -5040*S(2)   zeros(1,8);     % 0 2,3
zeros(1,16)       1          S(2)         S(2)^2         S(2)^3        S(2)^4         S(2)^5         S(2)^6        S(2)^7    zeros(1,8);                                                                                                                         % p3 initial point 3
zeros(1,16)       1          S(3)         S(3)^2         S(3)^3        S(3)^4         S(3)^5         S(3)^6        S(3)^7    zeros(1,8);                                                                                                                         % p4 final point 3
zeros(1,16)       0           1           2*S(3)        3*S(3)^2      4*S(3)^3       5*S(3)^4      6*S(3)^5      7*S(3)^6     -0           -1           -2*S(3)        -3*S(3)^2      -4*S(3)^3       -5*S(3)^4      -6*S(3)^5      -7*S(3)^6    ;               % 0 3,4
zeros(1,16)       0           0             2           6*S(3)        12*S(3)^2     20*S(3)^3      30*S(3)^4     42*S(3)^5    -0           -0             -2           -6*S(3)        -12*S(3)^2     -20*S(3)^3      -30*S(3)^4     -42*S(3)^5   ;               % 0 3,4
zeros(1,16)       0           0             0             6           24*S(3)       60*S(3)^2     120*S(3)^3     210*S(3)^4   -0            -0             -0             -6           -24*S(3)       -60*S(3)^2     -120*S(3)^3     -210*S(3)^4 ;               % 0 3,4
zeros(1,16)       0           0             0             0            24            120*S(3)     360*S(3)^2     840*S(3)^3   -0           -0             -0             -0            -24            -120*S(3)     -360*S(3)^2     -840*S(3)^3  ;               % 0 3,4
zeros(1,16)       0           0             0             0             0              120        720*S(3)       2520*S(3)^2  -0           -0             -0             -0             -0              -120        -720*S(3)       -2520*S(3)^2 ;               % 0 3,4
zeros(1,16)       0           0             0             0             0               0           720          5040*S(3)    -0           -0             -0             -0             -0               -0           -720          -5040*S(3)   ;               % 0 3,4
zeros(1,24)       1          S(3)         S(3)^2         S(3)^3        S(3)^4         S(3)^5         S(3)^6        S(3)^7   ;                                                                                                                                    % p4 initial point 4
zeros(1,24)       1          S(4)         S(4)^2         S(4)^3        S(4)^4         S(4)^5         S(4)^6        S(4)^7   ;                                                                                                                                    % p5 final point 4
zeros(1,24)       0           1           2*S(4)        3*S(4)^2      4*S(4)^3       5*S(4)^4      6*S(4)^5      7*S(4)^6    ;                                                                                                                                   % 0 end velocity zero
zeros(1,24)       0           0             2           6*S(4)        12*S(4)^2     20*S(4)^3      30*S(4)^4     42*S(4)^5   ;                                                                                                                                   % 0 end accel zero
zeros(1,24)       0           0             0             6           24*S(4)       60*S(4)^2     120*S(4)^3     210*S(4)^4  ];                                                                                                                                  % 0 end jerk zero    

X = [waypoints0(1,1) ; waypoints0(1,2); zeros(9,1); waypoints0(1,2); waypoints0(1,3); zeros(6,1); waypoints0(1,3); waypoints0(1,4); zeros(6,1); waypoints0(1,4); waypoints0(1,5); 0; 0; 0];
Y = [waypoints0(2,1) ; waypoints0(2,2); zeros(9,1); waypoints0(2,2); waypoints0(2,3); zeros(6,1); waypoints0(2,3); waypoints0(2,4); zeros(6,1); waypoints0(2,4); waypoints0(2,5); 0; 0; 0];
Z = [waypoints0(3,1) ; waypoints0(3,2); zeros(9,1); waypoints0(3,2); waypoints0(3,3); zeros(6,1); waypoints0(3,3); waypoints0(3,4); zeros(6,1); waypoints0(3,4); waypoints0(3,5); 0; 0; 0];
CoefX = inv(C)*X;
CoefY = inv(C)*Y;
CoefZ = inv(C)*Z;

CX = zeros(4,8);
CX(1,:) = CoefX(1:8);
CX(2,:) = CoefX(9:16);
CX(3,:) = CoefX(17:24);
CX(4,:) = CoefX(25:32);

CY = zeros(4,8);
CY(1,:) = CoefY(1:8);
CY(2,:) = CoefY(9:16);
CY(3,:) = CoefY(17:24);
CY(4,:) = CoefY(25:32);

CZ = zeros(4,8);
CZ(1,:) = CoefZ(1:8);
CZ(2,:) = CoefZ(9:16);
CZ(3,:) = CoefZ(17:24);
CZ(4,:) = CoefZ(25:32);


for t = 0:0.01:15
    if t<=S(1)
        i = 1;
    elseif t<=S(2);
        i=2;
    elseif t<=S(3);
        i=3;
    else i=4;
    end
    v1 = [1 t t^2 t^3 t^4 t^5 t^6 t^7]';
    v2 = [0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6]';
    v3 = [0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5]';
    desired_state.pos = zeros(3,1);
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.pos(1,1) = CX(i,:)*v1;
    desired_state.pos(2,1) = CY(i,:)*v1;
    desired_state.pos(3,1) = CZ(i,:)*v1;
    
    desired_state.vel(1,1) = CX(i,:)*v2;
    desired_state.vel(2,1) = CY(i,:)*v2;
    desired_state.vel(3,1) = CZ(i,:)*v2;
    
    desired_state.acc(1,1) = CX(i,:)*v3;
    desired_state.acc(2,1) = CY(i,:)*v3;
    desired_state.acc(3,1) = CZ(i,:)*v3;
    
    desired_state.yaw = 0;
    desired_state.yawdot = 0;

    plot3(desired_state.pos(1,1), desired_state.pos(2,1), desired_state.pos(3,1),'.')
    hold on;
end
%%
% for t=4:0.01:8
%     figure(1)
%     plot(t, CX(2,:)*[1 t t^2 t^3 t^4 t^5 t^6 t^7]');
%     hold on;
%     figure(2);
%     plot(t,CY(2,:)*[1 t t^2 t^3 t^4 t^5 t^6 t^7]','r');
%     hold on;
%     figure(3);
%     plot(t,CZ(2,:)*[1 t t^2 t^3 t^4 t^5 t^6 t^7]','k');
%     hold on;
% end









