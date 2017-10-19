function solveConsOpt()

x0 = [0.1,-0.1];
A = [];
b = [];

options = optimset('Display','off');
[xsol,fval,exitflag,output] = fmincon(@(x) costfun(x), x0, ...
                                    A, b, [], [], ...
                                    [], [], ...
                                    @(x) constfun(x), options);
xsol
fval
exitflag
printSolution(exitflag)

c = 1/xsol(1) - xsol(2)

end

function cost = costfun(x)
    cost = x(1)^2 + x(2)^2;
end

function [c,ceq] = constfun(x)
c = [];
ceq = [];

c(1) = 1/abs(x(1)) - abs(x(2));

end

function printSolution(exitflag)
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