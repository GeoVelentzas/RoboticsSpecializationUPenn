function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

persistent gx gy
[gx, gy] = gradient(-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
GX = gx';
GY = gy';
iterations = 0;
pos = start_coords;
route = pos;

% while (iterations <= max_its) && (abs(pos(1)-end_coords(1)) + abs(pos(2)-end_coords(2)))>=2
%     mx = (GX(pos(1),pos(2)));
%     my = (GY(pos(1),pos(2)));
%     if norm(mx)>norm(my)
%         v = [sign(mx) 0];
%     else
%         v = [0 sign(my)];
%     end
% 
%     pos = pos + v;
%     route = [route; pos];
%     iterations = iterations+1;
%     
% end

while(iterations<=max_its) && (norm(pos-end_coords)>=2)
    pp = round(pos);
    v = [GX(pp(1),pp(2)) GY(pp(1),pp(2))];
    v = v/norm(v);
    pos = pos+v;
    route = [route; pos];
    iterations = iterations+1;
end



% *******************************************************************
end
