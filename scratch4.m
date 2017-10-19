function scratch4()

x = 1;
cost = costfunction(@runningcosts,x);

disp(cost)                

disp('done!')

end

function cost = costfunction(runningcosts,x)
                
cost = 2*runningcosts(x);

end

function cost  = runningcosts(x)
    cost = x;
end

%function x = computeOpenloopSolution(system, N, T, t0, x0, u)
%    x = 0;
%end

function xkp1 = system(xk, uk, T)
    xkp1 = xk + T*uk;
end



