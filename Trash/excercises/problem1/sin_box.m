function f = sin_box(x,Xfin,iter)
    f = 0; % initialization
    for ii = 1:iter
        f = sin((2*ii-1)*2*pi*x/(2*Xfin))/(2*ii-1) + f;
    end
    f = f + 4;
end