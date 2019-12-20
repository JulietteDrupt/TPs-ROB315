function VisualisationEllipsoide(J, q, g_0E)

    M = J*J.';
    [vecteurs_propres,valeurs_propres]=eig(M);
    
    VisualisationRepere(q);
    [angle,axe] = rotationParams(vecteurs_propres);
    
    positionFinale = g_0E(1:3,4);
    
    [x, y, z] = ellipsoid(positionFinale(1),positionFinale(2),positionFinale(3),valeurs_propres(1,1),valeurs_propres(2,2),valeurs_propres(3,3));
    S = surf(x, y, z,'FaceAlpha',0.5);
    rotate(S,axe,angle*180/pi,positionFinale);
    
    axis equal;
    hold on;
end