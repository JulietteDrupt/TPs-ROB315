function[qd] = MCI(Xdi,Xdf,V,Te,qi)

    global kmax epsilon alphaStep; 

    distance = norm(Xdf - Xdi);
    ds = V*Te;
    
    Xd = [Xdi];
    qd = [qi];
    qdk = qi;
    Xdk = Xdi;
    
    for i = 1:distance/ds
        Xdk = Xdk+ds*(Xdf-Xdi)/distance;
        Xd = [Xd Xdk];
        
        qdk = MGI(Xdk,qdk,kmax,epsilon,alphaStep);
        qd = [qd, qdk];
    end
end