function N  = Neighbors( current,input_map )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

N = [];
[nrows, ncols] = size(input_map);
[i, j] = ind2sub(size(input_map), current);

left = [i,j-1];
right = [i,j+1];
up = [i-1,j];
down = [i+1,j];

if (right(2)<=ncols)
    N= [N right'];
end
if (down(1)<=nrows) 
    N = [N down'];
end
if (up(1)>=1) 
    N = [N up'];
end
if (left(2)>=1)
    N = [N left'];
end





    
end

