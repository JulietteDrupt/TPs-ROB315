%>% ======================================================================
%>%> @brief Résout le Modèle Géométrique Inverse en utilisant une méthode
%>%> itérative basée sur la pseudo-inverse de la matrice jacobienne (slide
%>%> 108 du cours) - Restriction aux seules vitesses de translation
%>%> 
%>%> @param Xd Position cartésienne désirée pour l'organe terminal 3x1
%>%> @param q0 Configuration articulaire initiale de la méthode itérative
%>%> nombre_articulationsx1
%>%> @param kmax Nombre maximal d'itérations 1x1
%>%> @param epsilon Critère d'arrêt sur l'erreur en position de l'organe
%>%> terminal 1x1
%>%> @param alphaStep [Optionnel] Facteur utilisé pour la méthode de descente 
%>%> du gradient basée sur la transposée de la matrice jacobienne
%>%>
%>%> @retval q Configuration articulaire solution nombre_articulationx1
%>% ======================================================================
function [q] = MGI(Xd,q0,kmax,epsilon,alphaStep)

    global alpha d r rE;

    theta = [q0(1) q0(2) q0(3)+pi/2 q0(4) q0(5) q0(6)];
    
    %Calcul du MGD
    [g_06,~] = CalculMGD(alpha,d,theta,r);
    g_0E = g_06*CalculTransformationElem(0,0,0,rE);
    
    k = 0;
    %Iterations
    while(norm(Xd - g_0E(1:3,4)) > epsilon && k < kmax)
        k = k+1;
        
        %Calcul de la matrice jacobienne
        J = CalculJacobienne(alpha,d,theta,r);
        J = J(1:3,:);
        
        %Calcul itératif de la configuration articulaire
        %q0 = q0 + alphaStep*J.'*(Xd - g_0E(1:3,4));
        q0 = q0 + pinv(J)*(Xd - g_0E(1:3,4));
        
        %Calcul du MGD
        theta = [q0(1) q0(2) q0(3)+pi/2 q0(4) q0(5) q0(6)];
        [g_06,~] = CalculMGD(alpha,d,theta,r);
        g_0E = g_06*CalculTransformationElem(0,0,0,rE);  
    end
    q = q0;
end