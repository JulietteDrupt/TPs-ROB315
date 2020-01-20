%>% ======================================================================
%>%> @brief Calcule le Modèle Géométrique Direct en utilisant les matrices
%>%> de transformations homogènes issues de la convention DHM
%>%> (slide 72 du cours)
%>%> 
%>%> @param alpha,d,theta,r Liste des paramètres de la convention DHM 
%>%> nombre_articulationsx1
%>%>
%>%> @retval g_06 Matrice de transformation homogène entre le repère R0 et
%>%> le repère R6
%>%> @retval g_elem Matrices de transformations homogènes entre les repères
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