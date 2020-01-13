function [OJv_Gi, OJ_wi] = CalculMatriceJacobienneGi(alpha, d, theta, r, x_G, y_G, z_G)

    %Global variables => Doesn't work well with simulink
    %global rE;
    
    rE = 0.1;

    OJ_OE = CalculJacobienne(alpha, d, theta, r);

    OJv_Gi = zeros(3,size(OJ_OE,2),size(OJ_OE,2));
    OJ_wi = zeros(3,size(OJ_OE,2),size(OJ_OE,2));
    OJ_Gi = zeros(size(OJ_OE,1),size(OJ_OE,2),size(OJ_OE,2));

    g_0i = eye(4);

    [g_06,~] = CalculMGD(alpha,d,theta,r);
    g_6E = CalculTransformationElem(0,0,0,rE);
    g_0E = g_06*g_6E;
    OO0OE = g_0E(1:3,4); OOEO0 = -OO0OE;

    for i=1:size(OJ_OE,2)
        
        OJ_Oi = OJ_OE(:,1:i);
        iOiGi = [x_G(i), y_G(i), z_G(i)]';
        
        g_elem = CalculTransformationElem(alpha(i),d(i),theta(i),r(i));
        g_0i = g_0i*g_elem;
        ROi = g_0i(1:3,1:3);
        
        OOiGi = ROi*iOiGi;
        
        OO0Oi = g_0i(1:3,4);
        OOEOi = OOEO0 + OO0Oi;
        OOEGi = OOEOi + OOiGi;
        
        OPreproduitVect_OEGi = [ 0 -OOEGi(3) OOEGi(2);
                                OOEGi(3) 0 -OOEGi(1) ;
                                -OOEGi(2) OOEGi(1) 0 ];

        OJ_Gi(:,1:i,i) = OJ_Gi(:,1:i,i) + [eye(3), -OPreproduitVect_OEGi; zeros(3,3), eye(3)]*OJ_Oi;
        
        OJv_Gi(:,:,i) = OJ_Gi(1:3,:,i);
        OJ_wi(:,:,i) = OJ_Gi(4:6,:,i);
    end
end

