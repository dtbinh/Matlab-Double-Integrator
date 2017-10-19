function [  ] = scratch2(  )

t0 = 0;
x0 = 1;
u = 1;
T = 0.1;

x = dynamic(t0,x0,u,T);
disp(x)

end

function xkp1 = system(t0, xk, uk, T)
    xkp1 = xk + T*uk;
end

function [x, t_intermediate, x_intermediate] = dynamic(t0,x0,u,T)

type = 'differential equation';
if ( strcmp(type, 'difference equation') )
    x = system(t0, x0, u, T);
    x_intermediate = [x0; x];
    t_intermediate = [t0, t0+T];
elseif ( strcmp(type, 'differential equation') )
    %options = odeset('AbsTol', atol_ode, 'RelTol', rtol_ode);
    options = [];
    [t_intermediate,x_intermediate] = ode45(system, ...
        [t0, t0+T], x0, u);
    x = x_intermediate(size(x_intermediate,1),:);
end
        
end

function x = computeOpenloopSolution(system, N, T, t0, x0, u)
                                 
    x(1,:) = x0;
    for k=1:N
        x(k+1,:) = dynamic(system, T, t0, x(k,:), u(:,k), ...
                             atol_ode_sim, rtol_ode_sim, type);
    end
    
end




