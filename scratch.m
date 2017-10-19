function scratch()

N = 10;
T = 0.1;
t0 = 0;
x0 = 0;

cost = costfunction(runningcosts, system, ...
                    N, T, t0, x0, u);

disp(cost)                

end

function cost = costfunction(runningcosts, system, ...
                    N, T, t0, x0, u)
                
for k=1:N                                
    costvec(k) = runningcosts(t0+k*T, x(k,:), u(:,k));      
    cost = cost+costvec(k);
end

end

function cost  = runningcosts(t, x, u)
    cost = 0;
end

%function x = computeOpenloopSolution(system, N, T, t0, x0, u)
%    x = 0;
%end

function xkp1 = system(xk, uk, T)
    xkp1 = xk + T*uk;
end



