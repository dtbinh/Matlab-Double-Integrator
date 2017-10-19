function [  ] = scratch7(  )

xk = [0; 0];
uk = 1;
T = 0.1;
for k = 1:101
    xkp1 = system(xk,uk,T);
    %disp(xk)
    xk = xkp1;
end

end

function xkp1 = system(xk, uk, T)
xkp1(1) = xk(1) + T*xk(2);
xkp1(2) = xk(2) + T*uk;
end

function myplot(t,x,u)
figure(1);
hold on
plot(t,x,'-');
figure(2);
hold on
plot(t,u,'-');
end
