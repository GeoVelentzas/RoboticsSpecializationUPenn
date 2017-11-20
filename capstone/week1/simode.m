
function Xend = simode(beta, tend)
  % this parameter is set by the calling script
  params.delta = 1;
  params.alpha = 1;
  params.beta = beta;
  params.gamma = 1;
  params.omega = 1;
  
  X0 = [1,1];
  tspan = [0 tend];  
  [~,X] = ode45(@(t,X) dyn(params,t,X), tspan, X0);
%   plot(X(:,1), X(:,2))
  comet(X(:,1), X(:,2));
  Xend = X(end,:);
end

function Xd = dyn(params, t, X)
  x = X(1);
  xd = X(2);

  Xd = zeros(size(X));
  Xd(1) = xd;
  Xd(2) = params.gamma*cos(params.omega*t)-params.delta*xd ...
      - params.alpha*x - params.beta*x^3;
end

