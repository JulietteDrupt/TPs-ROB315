%>% ======================================================================
%>%> @brief Retourne la matrice de transformation homogène correspondant
%>%> aux quatre paramètres de la convention DHM (slide 71 du cours)
%>%> 
%>%> @param alphai,di,thetai,ri Paramètres de la convention DHM 1x1
%>%>
%>%> @retval g Matrice de transformation homogène 4x4
%>% ======================================================================
function [g] = CalculTransformationElem(alphai,di,thetai,ri)
    g = [cos(thetai) -sin(thetai) 0 di;
        cos(alphai)*sin(thetai) cos(alphai)*cos(thetai) -sin(alphai) -ri*sin(alphai);
        sin(alphai)*sin(thetai) sin(alphai)*cos(thetai) cos(alphai) ri*cos(alphai);
        0 0 0 1];
end