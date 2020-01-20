%>% ======================================================================
%>%> @brief Retourne un visuel de l'ellipsoide de transmission en vitesse
%>%> associée à la matrice jacobienne J calculée dans la configuration
%>%> articulaire q (slide 93 du cours)
%>%> 
%>%> @param J Matrice jacobienne dimensions_espace_operationnelx
%>%> nombre_articulations
%>%> @param q Configuration articulaire nombre_articulationsx1
%>%> @param g_0E Matrice de transformation homogène du repère RE dans R0
%>%> 4x4
%>% ======================================================================
function VisualisationEllipsoide(J, q, g_0E)

    %Calcul des valeurs singulières de J
    M = J*J.';
    [vecteurs_propres,valeurs_propres]=eig(M);
    valeurs_singulieres = sort(sqrt(diag(valeurs_propres)));
    
    VisualisationRepere(q);
    
    %Calcul de la position et de l'orientation de l'ellipsoide
    [angle,axe] = rotationParams(vecteurs_propres);
    positionFinale = g_0E(1:3,4);
    
    %Dessin de l'ellipsoide
    [x, y, z] = ellipsoid(positionFinale(1),positionFinale(2),positionFinale(3),valeurs_singulieres(1),valeurs_singulieres(2),valeurs_singulieres(3));
    S = surf(x, y, z,'FaceAlpha',0.5);
    rotate(S,axe,angle*180/pi,positionFinale);
    
    axis equal;
    grid on;
    hold on;
end