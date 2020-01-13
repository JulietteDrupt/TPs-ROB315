function[tf] = CalculTempsTrajectoire(qdi,qdf,mu2)    
    Tmax_m = ones(size(qdi))*5;
    Rred = [100 100 100 70 70 70].';
    Tmax_a = Tmax_m.*Rred;
    
    ka = Tmax_a/mu2;
    
    D = qdf - qdi;
    
    tf = sqrt(10*D./(sqrt(3)*ka));
end