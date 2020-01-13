function A = CalculMatriceInertie(q)

    global alpha d r m OG I Rred Jm;
    
    theta = [q(1) q(2) q(3)+pi/2 q(4) q(5) q(6)].';
    g_0i = eye(4);

    Ig = zeros(3*size(q,1),3);
    for i = 1:length(m)
        Huy = [OG(i,2)^2+OG(i,3)^2 -OG(i,1)*OG(i,2) -OG(i,1)*OG(i,3);
                -OG(i,1)*OG(i,2) OG(i,1)^2+OG(i,3)^2 -OG(i,2)*OG(i,3);
                -OG(i,1)*OG(i,3) -OG(i,2)*OG(i,3) OG(i,1)^2+OG(i,2)^2];
        
        g_elem = CalculTransformationElem(alpha(i),d(i),theta(i),r(i));
        g_0i = g_0i*g_elem;
        ROi = g_0i(1:3,1:3);

        Ig(3*(i-1)+1:3*i,:) = ROi*(I(3*(i-1)+1:3*i,:)-m(i).*Huy)*ROi.';        
    end
    
    [OJv_Gi, OJ_wi] = CalculMatriceJacobienneGi(alpha, d, theta, r, OG(:,1), OG(:,2), OG(:,3));

    A = zeros(size(theta,2),size(theta,2));
    for i=1:size(theta,2) 
        A = A + OJv_Gi(:,:,i).'*OJv_Gi(:,:,i)*m(i)+ OJ_wi(:,:,i).'*Ig(3*(i-1)+1:3*i,:)*OJ_wi(:,:,i);
    end
   
    A = A + diag(Rred.^2.*Jm);

end

