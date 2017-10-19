function scratch5()

N = 5;
T = 0.1;
t0 = 0;
x0 = 1;
u = ones(1,N);

cost = costfunction(@runningcosts, @system, N, T, t0, x0, u);
disp(cost)                

u0 = u;
x = computeOpenloopSolution(@system, N, T, t0, x0, u0);
disp(x)                            

A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

u = fmincon(@(u) costfunction(@runningcosts, @system, N, T, t0, x0, u), ...
        u0, A, b, Aeq, beq, lb, ub, []);

%u = fmincon(@(u) costfunction(@runningcosts, @system, N, T, t0, x0, u), u0);
disp(u);

x_new = computeOpenloopSolution(@system, N, T, t0, x0, u);
disp(x_new);

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
    Q = eye(2);
    R = 1;
    x = x(:);
    u = u(:);
    cost = x'*Q*x + u'*R*u;
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u)
   x0 = [0 0];
   x(1,:) = x0;
   for k=1:N
       x(k+1,:) = system(x(k,:), u(:,k), T);
   end
end

function xkp1 = system(xk, uk, T)
    xkp1(1) = xk(1) + T*uk;
    xkp1(2) = xk(2) + T*uk;
end


