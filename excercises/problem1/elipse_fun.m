function elp = elipse_fun(x,y,Xa,Ya,R1,R2)
	elp = - ((x-Xa)/R1).^6 - (y-Ya/R2).^6 + 3;
end
