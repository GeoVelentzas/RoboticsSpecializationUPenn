function qdd = eom(params, th, phi, dth, dphi, u)
  % This is the starter file for the week5 assignment

  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius

  % Provided states are:
  % th: wheel angle (relative to body)
  % phi: body pitch
  % dth, dphi: time-derivatives of above
  % u: torque applied at the wheel
    
  r = params.r;
  g = params.g;
  mb = params.mr;
  m = mb;
  ib = params.ir;
  l = params.d;
  theta = th;
  phidot = dphi;
  thetadot = dth;
  
%   dLtheta = 0;
%   dLphi = g*l*mb*sin(phi) - (mb*(2*l*phidot*sin(phi)*(r*(phidot + thetadot) + l*phidot*cos(phi)) - 2*l^2*phidot^2*cos(phi)*sin(phi)))/2;
%   dLthetadot = mb*r*(r*(phidot + thetadot) + l*phidot*cos(phi));
%   dLphidot = ib*phidot + (mb*(2*(r + l*cos(phi))*(r*(phidot + thetadot) + l*phidot*cos(phi)) + 2*l^2*phidot*sin(phi)^2))/2
%   dLthetadotdt = mb*r*(r*(phiddot + thetaddot) + l*cos(phi)*phiddot - l*sin(phi)*phidot^2);
%   dLphidotdt = ib*phiddot + (mb*((2*r + 2*l*cos(phi))*(r*(phiddot + thetaddot) + l*cos(phi)*phiddot - l*sin(phi)*phidot^2) + 2*l^2*sin(phi)^2*phiddot - 2*l*sin(phi)*(r*(phidot + thetadot) + l*cos(phi)*phidot)*phidot + 4*l^2*cos(phi)*sin(phi)*phidot*phidot))/2;

  
  
%   dLphi = -mb*l*phidot*sin(phi)*(r*(thetadot+phidot)+l*phidot*cos(phi))+mb*l^2+phidot^2*sin(phi)*cos(phi)+mb*g*l*sin(phi);
%   A = zeros(2,2);
%   B = zeros(2,1);
%   
%   
  
%   A(1,1) = mb*r^2;
%   A(2,1) = mb*r^2;
%   A(1,2) = mb*r^2+mb*r*l*cos(phi);
%   A(2,2) = mb*r^2*l*cos(phi)+mb*r^2+mb*l^2*sin(phi)^2+ib;
%   
%   B(1) = -u + mb*r*l*phidot^2*sin(phi);
%   B(2) = dLphi + mb*r*l*phidot^2*sin(phi)-mb*l^2*phidot^2*2*sin(phi)*cos(phi);
%   
%   
  
A = [ mb*r^2,                                                        mb*r*(r + l*cos(phi));
     (mb*r*(2*r + 2*l*cos(phi)))/2,         ib + (mb*((2*r + 2*l*cos(phi))*(r + l*cos(phi)) + 2*l^2*sin(phi)^2))/2];  
  

 B =  [                                                                                                                                                                                                               l*mb*r*sin(phi)*phidot^2 + u;
 (mb*(2*l*sin(phi)*phidot*(r*(phidot + thetadot) + l*cos(phi)*phidot) - 4*l^2*cos(phi)*sin(phi)*phidot^2 + l*sin(phi)*phidot^2*(2*r + 2*l*cos(phi))))/2 + (mb*(2*l^2*cos(phi)*sin(phi)*phidot^2 - 2*l*sin(phi)*phidot*(r*(phidot + thetadot) + l*cos(phi)*phidot)))/2 + g*l*mb*sin(phi)];   

 qdd = linsolve(A,B);
  
%   qdd = [0;0];
  % THE STUDENT WILL FILL THIS OUT
end




