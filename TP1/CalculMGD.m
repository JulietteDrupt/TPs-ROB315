%>% ======================================================================
%>%> @brief Calcule le Mod�le G�om�trique Direct en utilisant les matrices
%>%> de transformations homog�nes issues de la convention DHM
%>%> (slide 72 du cours)
%>%> 
%>%> @param alpha,d,theta,r Liste des param�tres de la convention DHM 
%>%> nombre_articulationsx1
%>%>
%>%> @retval g_06 Matrice de transformation homog�ne entre le rep�re R0 et
%>%> le rep�re R6
%>%> @retval g_elem Matrices de transformations homog�nes entre les rep�res
%>%> R(i-1) et R(i) cell(1,nombre_articulations)
%>% ======================================================================
function [g_06,g_elem] = CalculMGD(alpha,d,theta,r)

    %Intialisation des tableaux
    g_06 = eye(4);
    g_elem = cell(1,length(alpha));

    for i=1:length(alpha)
        g_elem{i} = CalculTransformationElem(alpha(i),d(i),theta(i),r(i));
        g_06 = g_06*g_elem{i};
    end 
end