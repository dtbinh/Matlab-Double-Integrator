function solveProblemCons()

N = 8;
T = 0.1;
t0 = 0;
x0 = [1 1]*0;
u0 = ones(1,N);
mpciterations = 31;

tmeasure = t0;
xmeasure = x0;

t = [];
x = [];
u = [];

mpciter = 0;
while (mpciter < mpciterations)
    
    % get new initial value
    [t0, x0] = measureInitialValue(tmeasure, xmeasure);
    
    % debug
    %u_test = ones(1,8)
    %[c,ceq] = nonlinearconstraints(@constraints, ...
    %    @terminalconstraints, @system, ...
    %    N, T, t0, x0, u_test);
    
    % solve optimal control problem
    [u_new, V, exitflag, output] = solveOptimalControlProblem ...
        (N, t0, x0, u0, T);
    
    % print solution
    printSolution(mpciter, exitflag)
    
    % store closed loop data
    t = [ t; tmeasure ];
    x = [ x; xmeasure ];
    u = [ u; u_new(:,1) ];
    
    % apply control
    [tmeasure, xmeasure] = applyControl(T, t0, x0, u_new);
    
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


function cost = costfunction(N, T, t0, x0, u)

cost = 0;
x = computeOpenloopSolution(N, T, t0, x0, u);

for k=1:N
    costvec(k) = runningcosts(t0+k*T, x(k,:), u(:,k));
    cost = cost+costvec(k);
end

end


function cost  = runningcosts(t, x, u)
Q = 100;
R = 1;
x = x(:);
u = u(:);
r = 1;
cost = (r-x(1))'*Q*(r-x(1)) + u'*R*u;
end


function [c,ceq] = nonlinearconstraints...
                        (N, T, t0, x0, u)

x = computeOpenloopSolution(N, T, t0, x0, u);
c = [];
ceq = [];
for k=1:N
    [cnew, ceqnew] = constraints(t0+k*T,x(k,:),u(:,k));
    c = [c cnew];
    ceq = [ceq ceqnew];
end

[cnew, ceqnew] = terminalconstraints(t0+(N+1)*T,x(N+1,:));
c = [c cnew];
ceq = [ceq ceqnew];

end


function [c,ceq] = constraints(t, x, u)

c = [];
ceq = [];

c(1) =  x(1) - 1.0; %  x(1) < 1
c(1) = -x(1) - 1.0; % -x(1) > 1
c(2) =  x(2) - 1.0; %  x(2) < 1
c(2) = -x(2) - 1.0; % -x(2) > 1

end


function [c,ceq] = terminalconstraints(t, x)

c = [];
ceq = [];

end


function x = computeOpenloopSolution( N, T, t0, x0, u)
x(1,:) = x0;
for k=1:N
    x(k+1,:) = system(x(k,:), u(:,k), T);
end
end


function xkp1 = system(xk, uk, T)
xkp1(1) = xk(1) + T*xk(2);
xkp1(2) = xk(2) + T*uk;
end


function [u, V, exitflag, output] = solveOptimalControlProblem ...
                                    ( N, t0, x0, u0, T)

%x = computeOpenloopSolution(system, N, T, t0, x0, u0);

A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
% lb = -[3,zeros(1,N-1)];
% ub =  [3,zeros(1,N-1)];
lb = -3*ones(1,N);
ub =  3*ones(1,N);


options = optimset('Display','off','Algorithm', 'active-set');
%options = optimset('Display','off','Algorithm', 'interior-point');
%options = optimset('Display','off','Algorithm', 'sqp');

[u, V, exitflag, output] = fmincon(@(u) costfunction...
    (N, T, t0, x0, u), ...
    u0, A, b, Aeq, beq, lb, ub, ...
    @(u) nonlinearconstraints(N, T, t0, x0, u), ...
    options );

end


function [t0, x0] = measureInitialValue(tmeasure, xmeasure)
t0 = tmeasure;
x0 = xmeasure;
end


function u0 = shiftHorizon(u)
u0 = [u(:,2:size(u,2)) u(:,size(u,2))];
end


function [tapplied, xapplied] = applyControl(T, t0, x0, u)
xapplied = system(x0, u(:,1), T);
tapplied = t0+T;
end


function myplot(t,x,u)
close all
figure(1);
hold on
plot(t,x,'.-');
legend('2nd integrator','1st integrator')
grid on
figure(2);
hold on
plot(t,u,'.-');
legend('control')
xlabel('t [sec]')
grid on
end

function printSolution(mpciter, exitflag)
fprintf('%3d ',mpciter);
switch(exitflag)
    case -2
        fprintf(' Error F\n');   % no feasible solution
    case -1
        fprintf(' Error OT\n');  % output function terminated the algorithm
    case 0
        fprintf(' Warning IT\n'); % numner of iterations/funvals exceeded tol
    case 1
        fprintf(' Ok \n');
    case 2
        fprintf(' Warning TX\n'); % change in x was less than tol
    case 3
        fprintf(' Warning TJ\n'); % change in the objective function less than tol
    case 4
        fprintf(' Warning S\n'); % magnitude of the search direction less than tol
    case 5
        fprintf(' Warning D\n'); % magnitude of directional derivative less than tol     
end
% https://www.mathworks.com/help/optim/ug/when-the-solver-fails.html#br44i73
end
