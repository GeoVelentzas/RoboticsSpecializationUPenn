% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%load('GMM');
%a=load('model');
a = load('GMM');
mu = a.GMModel.mu(1,:)';
S = a.GMModel.Sigma(:,:,1);
%mu = a.mu;
%S = a.S;
% w = GMModel.ComponentProportion;
% mu = GMModel.mu;
% sig = GMModel.Sigma;
mu = reshape(mu, 3, 1);
Sinv = inv(S);
I = double(I);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
for i = 1:size(I, 1)
    for j=1:size(I,2)
        x = I(i,j,:);
        x = reshape(x, 3, 1);
        %prob(i,j) = 1/(2*pi)^(3/2)/det(S)^(1/2)*exp(-(x-mu)'*Sinv*(x-mu));
        prob(i,j) = -1/2*log(det(S))+(-(x-mu)'*Sinv*(x-mu));
    end
end
%imshow(prob>-15);

bw = prob>-17;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

bw_biggest = false(size(bw));
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
plot(loc(1), loc(2),'r+');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI = bw;

% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
