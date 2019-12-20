function [q] = MGI(Xd,q0,kmax,epsilon,alphaStep)

    global alpha d r rE;

    theta = [q0(1) q0(2) q0(3)+pi/2 q0(4) q0(5) q0(6)];
    [g_06,~] = CalculMGD(alpha,d,theta,r);
    g_0E = g_06*CalculTransformationElem(0,0,0,rE);
    
    k = 0;
    while(norm(Xd - g_0E(1:3,4)) > epsilon && k < kmax)
        k = k+1;
        J = CalculJacobienne(alpha,d,theta,r);
        J = J(1:3,:);
        %q0 = q0 + alphaStep*J.'*(Xd - g_0E(1:3,4));
        q0 = q0 + pinv(J)*(Xd - g_0E(1:3,4));
        
        theta = [q0(1) q0(2) q0(3)+pi/2 q0(4) q0(5) q0(6)];
        [g_06,~] = CalculMGD(alpha,d,theta,r);
        g_0E = g_06*CalculTransformationElem(0,0,0,rE);  
    end
    q = q0;
end