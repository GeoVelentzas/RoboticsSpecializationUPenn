% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization2(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

%%
%visualize some variables
% myResolution
% myOrigin
% size(myPose)
% N
% pause;
param.init_pose
pause;

%%

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 10;                       % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]); %now I have copies of my pose M-times
So = diag([0.01, 0.01, 0.01]);
Sm = diag([0.1, 0.1, 0.05]);
figure;
bar3(P);
pause;
% N is the number of timesteps... poses for which I have measurements

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 
    %maybe propagate probabilistically each pose from P matrix by adding
    %random noise to each particle drawn for gaussian distributions of
    %observational and model noise...
    add1 = randn(1,M)*So(1,1) + randn(1,M)*Sm(1,1);
    add2 = randn(1,M)*So(2,2) + randn(1,M)*Sm(2,2);
    add3 = randn(1,M)*So(3,3) + randn(1,M)*Sm(3,3);
    P = P + [add1; add2; add3];
    
    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
    for particle = 1:size(P,2);
        xrobotParticle = myPose(1,particle);
        yrobotParticle = myPose(2,particle);
        thetaParticle = myPose(3,particle);
        ixrobotParticle = ceil(myResolution*xrobot) + myOrigin(1);
        iyrobotParticle = ceil(myResolution*yrobot) + myOrigin(2);
        d = range(:,j);
        alpha = scanAngles;
        xoccupied = d.*cos(thetaParticle + alpha) + xrobotParticle;
        yoccupied = -d.*sin(thetaParticle + alpha) + yrobotParticle;
        ixoccupied = ceil(myResolution*xoccupied) + myOrigin(1);
        iyoccupied = ceil(myResolution*yoccupied) + myOrigin(2);

    %   2-2) For each particle, calculate the correlation scores of the particles

    %   2-3) Update the particle weights         
 
    %   2-4) Choose the best particle to update the pose
    
    % 3) Resample if the effective number of particles is smaller than a threshold

    % 4) Visualize the pose on the map as needed
   

end

end

