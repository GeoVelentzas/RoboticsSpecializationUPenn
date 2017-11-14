function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
flag = true;

%all points triangle 1
p11 = P1(1,:)';
p12 = P1(2,:)';
p13 = P1(3,:)';

%all points triangle 2
p21 = P2(1,:)';
p22 = P2(2,:)';
p23 = P2(3,:)';

%Triangle 1 segments
%segment p11 p12
p = p11;
q = p12;
r = p13;
s = p21-p;
t = p22-p;
u = p23-p;
r = r-p;
n = q-p;
n = [n(2) -n(1)]'; %normal vector of segment
if (sign(n'*s)==sign(n'*t)) && (sign(n'*s)==sign(n'*u)) && (sign(n'*s)~=sign(n'*r))
    flag = false;
end

%segment p11 p13
p = p11;
q = p13;
r = p12;
s = p21-p;
t = p22-p;
u = p23-p;
r = r-p;
n = q-p;
n = [n(2) -n(1)]'; %normal vector of segment
if (sign(n'*s)==sign(n'*t)) && (sign(n'*s)==sign(n'*u)) && (sign(n'*s)~=sign(n'*r))
    flag = false;
end

%segment p12 p13
p = p13;
q = p12;
r = p11;
s = p21-p;
t = p22-p;
u = p23-p;
r = r-p;
n = q-p;
n = [n(2) -n(1)]'; %normal vector of segment
if (sign(n'*s)==sign(n'*t)) && (sign(n'*s)==sign(n'*u)) && (sign(n'*s)~=sign(n'*r))
    flag = false;
end

%Triangle 2 segments
%segment p21 p22
p = p21;
q = p22;
r = p23;
s = p11-p;
t = p12-p;
u = p13-p;
r = r-p;
n = q-p;
n = [n(2) -n(1)]'; %normal vector of segment
if (sign(n'*s)==sign(n'*t)) && (sign(n'*s)==sign(n'*u)) && (sign(n'*s)~=sign(n'*r))
    flag = false;
end

%segment p22 p23
p = p22;
q = p23;
r = p21;
s = p11-p;
t = p12-p;
u = p13-p;
r = r-p;
n = q-p;
n = [n(2) -n(1)]'; %normal vector of segment
if (sign(n'*s)==sign(n'*t)) && (sign(n'*s)==sign(n'*u)) && (sign(n'*s)~=sign(n'*r))
    flag = false;
end

%segment p21 p23
p = p21;
q = p23;
r = p22;
s = p11-p;
t = p12-p;
u = p13-p;
r = r-p;
n = q-p;
n = [n(2) -n(1)]'; %normal vector of segment
if (sign(n'*s)==sign(n'*t)) && (sign(n'*s)==sign(n'*u)) && (sign(n'*s)~=sign(n'*r))
    flag = false;
end



% *******************************************************************
end