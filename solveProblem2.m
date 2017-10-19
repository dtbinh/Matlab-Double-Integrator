function solveProblem2()

N = 8;
T = 0.1;
t0 = 0;
x0 = [1 1]*0;
u0 = ones(1,N);
mpciterations = 100;

tmeasure = t0;
xmeasure = x0;

t = [];
x = [];
u = [];

mpciter = 0;
while (mpciter < mpciterations)
    
    % get new initial value
    [t0, x0] = measureInitialValue(tmeasure, xmeasure);
    
    % solve optimal control problem
    [u_new, V, exitflag, output] = solveOptimalControlProblem ...
        (@runningcosts, @system, N, t0, x0, u0, T);
    
    % store closed loop data
    t = [ t; tmeasure ];
    x = [ x; xmeasure ];
    u = [ u; u_new(:,1) ];
    
    % apply control
    [tmeasure, xmeasure] = applyControl(@system, T, t0, x0, u_new);
    
    % prepare restart
    u0 = shiftHorizon(u_new);
    
    mpciter = mpciter+1;
end

% disp('Final output:');
% disp(u_new);
% disp(V);
% disp(exitflag);
% disp(output);
myplot(t,x,u);

disp('done!')

end

function cost = costfunction(runningcosts, system, N, T, t0, x0, u)

cost = 0;
x = computeOpenloopSolution(system, N, T, t0, x0, u);

for k=1:N
    costvec(k) = runningcosts(t0+k*T, x(k,:), u(:,k));
    cost = cost+costvec(k);
end
end

function cost  = runningcosts(t, x, u)
%Q = eye(2)*20;
Q = 10;
R = 1;
x = x(:);
u = u(:);
r = [1];
%cost = (r-x)'*Q*(r-x) + u'*R*u;
cost = (r-x(2))'*Q*(r-x(2)) + u'*R*u;
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u)
x(1,:) = x0;
for k=1:N
    x(k+1,:) = system(x(k,:), u(:,k), T);
end
end

function xkp1 = system(xk, uk, T)
xkp1(1) = xk(1) + T*uk;
xkp1(2) = xk(2) + T*xk(1) + T*uk;
end

function [u, V, exitflag, output] = solveOptimalControlProblem ...
    (runningcosts, system, N, t0, x0, u0, T)

%x = computeOpenloopSolution(system, N, T, t0, x0, u0);

A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

options = optimset('Display','off');
[u, V, exitflag, output] = fmincon(@(u) costfunction...
    (runningcosts, system, N, T, t0, x0, u), ...
    u0, A, b, Aeq, beq, lb, ...
    ub, [], options );

end

function [t0, x0] = measureInitialValue(tmeasure, xmeasure)
t0 = tmeasure;
x0 = xmeasure;
end

function u0 = shiftHorizon(u)
u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end

function [tapplied, xapplied] = applyControl(system, T, t0, x0, u)
xapplied = system(x0, u(:,1), T);
%xapplied = system(x0,1,T);
tapplied = t0+T;
end

function myplot(t,x,u)
figure(1);
hold on
plot(t,x,'-');
figure(2);
hold on
plot(t,u,'-');
end
