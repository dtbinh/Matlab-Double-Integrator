function scratch8()

x = fsolve(@myfun,[-3 0.1 3],optimoptions('fsolve','Display','iter'))

keyboard
end


function F = myfun(x)
F = sin(x);

end