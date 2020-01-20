%>% ======================================================================
%>%> @brief Calcule le vecteur des couples dus à la gravité G(q) en
%>%> utilisant le calcul de l'énergie potentielle de pesanteur de la chaîne
%>%> articulée (slide 149 du cours)
%>%>
%>%> @param q Configuration articulaire nombre_articulationsx1
%>%>
%>%> @retval G Vecteur des couples induits par la gravité 
%>%> nombre_articulationsx1
%>% ======================================================================
function [G] = CalculCoupleGravite(q)
    g = [0 0 -9.81].';  
    
    %Variables globales => Ne fonctionnent pas bien avec Simulink
    %global alpha d r OG m;
    
    %Parametres geometriques du robot - Convention DHM
    d3 = 0.7;
    r1 = 0.5;
    r4 = 0.2;

    alpha = [0 pi/2 0 pi/2 -pi/2 pi/2];
    d = [0 0 d3 0 0 0];
    r = [r1 0 0 r4 0 0];

    %Parametres inertiels du robot

    m = [15 10 1 7 1 0.5];
    OG = [0 0 -0.25;0.35 0 0;0 -0.1 0; 0 0 0;0 0 0;0 0 0];
    
    theta = [q(1) q(2) q(3)+pi/2 q(4) q(5) q(6)].';
    
    %Calcul des matrices jacobienne relatives aux vitesses de translation aux centres de masses
    [OJv_Gi, ~] = CalculMatriceJacobienneGi(alpha, d, theta, r, OG(:,1),OG(:,2),OG(:,3));
    
    %Intialisation du tableau
    G = zeros(size(theta));
    
    for i=1:size(theta,1)
       G = G + OJv_Gi(:,:,i).'*m(i)*g;
    end
    
    G = -G;
end