%>% ======================================================================
%>%> @brief Calcule la matrice jacobienne en utilisant la méthode des
%>%> compositions de vitesse (slide 86 du cours)
%>%>
%>%> @param alpha,d,theta,r Liste des paramètres de la convention DHM 
%>%> nombre_articulationsx1
%>%>
%>%> @retval J Matrice jacobienne dimensions_espace_operationnelx
%>%> nombre_articulations
%>% ======================================================================
function [J] = CalculJacobienne(alpha,d,theta,r)

    %Variables globales => Ne fonctionne pas bien avec Simulink
    %global rE;
    
    rE =0.1;

    %Calcul du MGD
    [g_06,g_elem] = CalculMGD(alpha,d,theta,r);
    g_0E = g_06*CalculTransformationElem(0,0,0,rE);
    
    %Initialisation du tableau
    J = zeros(size(alpha,2),size(alpha,2));
    
    for i=1:length(alpha)
        %Calcul de la matrice de rotation entre R0 et R(i)
        g_0i = eye(4);
        
        for j=1:i
            g_0i = g_0i*g_elem{j};
        end
       
        R_0i = g_0i(1:3,1:3);
        
        %Calcul du vecteur OiOE dans le repère R0
        p_0E = g_0E(1:3,4);
        p_0i = g_0i(1:3,4);
        p_iE = p_0E - p_0i;
        
        J(:,i) = [cross(R_0i*[0 0 1].',p_iE);R_0i*[0 0 1].'];   
    end
end