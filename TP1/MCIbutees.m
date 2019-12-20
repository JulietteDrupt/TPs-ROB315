function [qd] = MCIbutees(Xdi,Xdf,V,Te,qi,alphaH,qmin,qmax)

    global kmax epsilon;

    distance = norm(Xdf - Xdi);
    ds = V*Te;
    
    Xd = [Xdi];
    qd = [qi];
    qdk = qi;
    Xdk = Xdi;
    
    for i = 1:distance/ds
        Xdk = Xdk+ds*(Xdf-Xdi)/distance;
        Xd = [Xd Xdk];
        
        theta = [qdk(1) qdk(2) qdk(3)+pi/2 qdk(4) qdk(5) qdk(6)];
        
        qdk = MGIbutees(Xdk,qdk,kmax,epsilon,qmin,qmax,alphaH);
        qd = [qd, qdk];
    end
end

function [q] = MGIbutees(Xd,q0,kmax,epsilon,qmin,qmax,alphaH)

    global alpha d r rE;

    theta = [q0(1) q0(2) q0(3)+pi/2 q0(4) q0(5) q0(6)];
    [g_06,~] = CalculMGD(alpha,d,theta,r);
    g_0E = g_06*CalculTransformationElem(0,0,0,rE);
    
    k = 0;
    while(norm(Xd - g_0E(1:3,4)) > epsilon && k < kmax)
        k = k+1;
        J = CalculJacobienne(alpha,d,theta,r);
        J = J(1:3,:);

        q0 = q0 + pinv(J)*(Xd - g_0E(1:3,4)) - alphaH*(eye(size(J,2)) - pinv(J)*J)*H(q0,qmax,qmin);
        
        theta = [q0(1) q0(2) q0(3)+pi/2 q0(4) q0(5) q0(6)];
        [g_06,~] = CalculMGD(alpha,d,theta,r);
        g_0E = g_06*CalculTransformationElem(0,0,0,rE);  
    end
    q = q0;
end

function [h] = H(q,qmax,qmin)
    h = [];
    dq = qmax-qmin;
    qbarre = dq/2;
    for i = 1:size(q,1)
        h = [h ; 2*(q(i)-qbarre(i))/dq(i)^2];
    end
end
