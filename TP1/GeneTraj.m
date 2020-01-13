function[qc] = GeneTraj(qdi,qdf,t)
    tf = 500;   %Temps en ms !
    D = qdf - qdi;
    
    r = 10*(t/tf)^3 - 15*(t/tf)^4 + 6*(t/tf)^5;
    
    qc = qdi + D*r;
end